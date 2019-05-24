#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/data/map_database.h"
#include "openvslam/map/local_mapper.h"
#include "openvslam/map/loop_closer.h"
#include "openvslam/match/projection.h"
#include "openvslam/match/bow_tree.h"
#include "openvslam/match/fuse.h"
#include "openvslam/optimize/global_bundle_adjuster.h"
#include "openvslam/solver/sim3_solver.h"
#include "openvslam/util/converter.h"

#include <mutex>
#include <thread>

#include <spdlog/spdlog.h>

namespace openvslam {
namespace map {

loop_closer::loop_closer(data::map_database* map_db, data::bow_database* bow_db, data::bow_vocabulary* bow_vocab, const bool fix_scale)
        : loop_detector_(bow_db, bow_vocab, fix_scale), map_db_(map_db),
          graph_optimizer_(map_db, fix_scale), fix_scale_(fix_scale) {
    spdlog::debug("CONSTRUCT: loop_closer");
}

loop_closer::~loop_closer() {
    abort_loop_BA();
    if (thread_for_loop_BA_) {
        thread_for_loop_BA_->join();
    }
    spdlog::debug("DESTRUCT: loop_closer");
}

void loop_closer::set_tracker(track::tracker* tracker) {
    tracker_ = tracker;
}

void loop_closer::set_local_mapper(map::local_mapper* local_mapper) {
    local_mapper_ = local_mapper;
}

void loop_closer::set_loop_detector_status(const bool loop_detector_is_enabled) {
    if (loop_detector_is_enabled) {
        spdlog::info("enable loop detector");
    }
    else {
        spdlog::info("disable loop detector");
    }
    loop_detector_is_enabled_ = loop_detector_is_enabled;
}

bool loop_closer::get_loop_detector_status() const {
    return loop_detector_is_enabled_;
}

void loop_closer::run() {
    spdlog::info("start loop closer");

    is_terminated_ = false;

    while (true) {
        if (keyframe_is_queued()) {
            {
                std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
                cur_keyfrm_ = keyfrms_queue_.front();
                keyfrms_queue_.pop_front();
                // ループ探索中はこのキーフレームの削除を禁止する
                cur_keyfrm_->set_not_to_be_erased();
            }
            if (loop_detector_.detect_loop_candidates(cur_keyfrm_)) {
                if (loop_detector_.validate_candidates()) {
                    correct_loop();
                }
            }
        }
        else if (check_and_execute_pause()) {
            // pauseフラグが立っていたらpause処理を行う
            // pauseされるまで待機，pauseされたままterminateされたらloop closerを終了する
            while (is_paused() && !check_terminate()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            if (check_terminate()) {
                break;
            }
        }

        // resetフラグが立っていたらreset処理を行う
        check_and_execute_reset();

        if (check_terminate()) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    terminate();
}

void loop_closer::queue_keyframe(data::keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    if (keyfrm->id_ != 0) {
        keyfrms_queue_.push_back(keyfrm);
    }
}

bool loop_closer::keyframe_is_queued() const {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    return (!keyfrms_queue_.empty());
}

void loop_closer::correct_loop() {
    auto final_candidate_keyfrm = loop_detector_.get_final_candidate_keyframe();

    spdlog::info("detect loop: keyframe {} - keyframe {}", final_candidate_keyfrm->id_, cur_keyfrm_->id_);
    ++num_exec_loop_BA_;

    // 0. ループ修正の前処理

    // 0-1. スレッド停止

    // loop correction中はlocal mapperを止める
    local_mapper_->request_pause();
    // 前回のloop BAがまだ動いていたら止める
    if (loop_BA_is_running() || thread_for_loop_BA_) {
        abort_loop_BA();
    }
    // local mapperが止まるまで待つ
    while (!local_mapper_->is_paused()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    // 0-2. グラフ情報の更新

    cur_keyfrm_->update_connections();

    // 1. current keyframeのSim3(修正後)を用いて，current keyframeのcovisibilitiesの修正前後のSim3を計算する
    //    修正前後の姿勢を用いて，covisibilitiesの姿勢およびそれらから観測できている3次元点も動かす

    // current keyframeとその周辺のkeyframeを集める
    std::vector<data::keyframe*> curr_covisibilities = cur_keyfrm_->get_covisibilities();
    curr_covisibilities.push_back(cur_keyfrm_);

    // 修正前のcovisibilitiesのSim3姿勢
    keyframe_Sim3_pairs_t non_corrected_Sim3s_iw;
    // 修正後のcovisibilitiesのSim3姿勢
    keyframe_Sim3_pairs_t pre_corrected_Sim3s_iw;

    const auto g2o_Sim3_world_to_curr = loop_detector_.get_g2o_Sim3_world_to_curr();
    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        // 修正前のcurrent keyframeの姿勢
        const Mat44_t cam_pose_curr_to_world = cur_keyfrm_->get_cam_pose_inv();

        // validate_candidates()で推定したg2o_Sim3_world_to_curr_(ループ修正後のcurrent keyframeのSim3)
        // を用いて，ループ修正前後のcurr_covisibilitiesのSim3を求める
        non_corrected_Sim3s_iw = get_non_corrected_Sim3s(curr_covisibilities);
        pre_corrected_Sim3s_iw = get_pre_corrected_Sim3s(cam_pose_curr_to_world, g2o_Sim3_world_to_curr, curr_covisibilities);

        // 修正前後の相対姿勢を使って3次元点を動かす
        correct_covisibility_landmarks(non_corrected_Sim3s_iw, pre_corrected_Sim3s_iw);
        // 修正後の姿勢に更新する
        correct_covisibility_keyframes(pre_corrected_Sim3s_iw);
    }

    // 3次元点の重複を解消する
    const auto curr_assoc_lms_in_cand = loop_detector_.get_curr_assoc_lms_in_cand();
    replace_duplicated_landmarks(curr_assoc_lms_in_cand, pre_corrected_Sim3s_iw);

    // curr_covisibilities_の各キーフレームに対して，ループ対応によって生じた新たなedgeを検出する
    const auto new_connections = extract_new_connections(curr_covisibilities);

    // pose graph optimization
    graph_optimizer_.optimize(final_candidate_keyfrm, cur_keyfrm_, non_corrected_Sim3s_iw, pre_corrected_Sim3s_iw, new_connections);

    // loop edgeを追加
    final_candidate_keyfrm->add_loop_edge(cur_keyfrm_);
    cur_keyfrm_->add_loop_edge(final_candidate_keyfrm);

    // loop BAを起動
    if (thread_for_loop_BA_) {
        thread_for_loop_BA_->join();
        thread_for_loop_BA_.reset(nullptr);
    }
    loop_BA_is_running_ = true;
    abort_loop_BA_ = false;
    thread_for_loop_BA_ = std::unique_ptr<std::thread>(new std::thread(&loop_closer::run_loop_BA, this, cur_keyfrm_->id_));

    // local mapperを再開
    local_mapper_->resume();

    loop_detector_.set_prev_loop_correct_keyfrm_id(cur_keyfrm_->id_);
}

keyframe_Sim3_pairs_t loop_closer::get_non_corrected_Sim3s(const std::vector<data::keyframe*>& covisibilities) const {
    // 修正前のcovisibilitiesのSim3姿勢
    keyframe_Sim3_pairs_t non_corrected_Sim3s_iw;

    for (auto covisibility : covisibilities) {
        // 修正前のcovisibilityの姿勢
        const Mat44_t cam_pose_world_to_covi = covisibility->get_cam_pose();
        // 修正前のworld->covisibilityのSim3を作って保存
        const Mat33_t& rot_world_to_covi = cam_pose_world_to_covi.block<3, 3>(0, 0);
        const Vec3_t& trans_world_to_covi = cam_pose_world_to_covi.block<3, 1>(0, 3);
        const g2o::Sim3 non_corrected_Sim3_world_to_covi(rot_world_to_covi, trans_world_to_covi, 1.0);
        non_corrected_Sim3s_iw[covisibility] = non_corrected_Sim3_world_to_covi;
    }

    return non_corrected_Sim3s_iw;
}

keyframe_Sim3_pairs_t loop_closer::get_pre_corrected_Sim3s(const Mat44_t& cam_pose_curr_to_world, const g2o::Sim3& g2o_Sim3_world_to_curr,
                                                           const std::vector<data::keyframe*>& covisibilities) const {
    // 修正後のcovisibilitiesのSim3姿勢
    keyframe_Sim3_pairs_t pre_corrected_Sim3s_iw;

    for (auto covisibility : covisibilities) {
        // 修正前のcovisibilityの姿勢
        const Mat44_t cam_pose_world_to_covi = covisibility->get_cam_pose();
        // まず，修正前の姿勢を用いてcurrent->covisibilityのSim3を作る
        const Mat44_t cam_pose_curr_to_covi = cam_pose_world_to_covi * cam_pose_curr_to_world;
        const Mat33_t& rot_curr_to_covi = cam_pose_curr_to_covi.block<3, 3>(0, 0);
        const Vec3_t& trans_curr_to_covi = cam_pose_curr_to_covi.block<3, 1>(0, 3);
        const g2o::Sim3 Sim3_curr_to_covi(rot_curr_to_covi, trans_curr_to_covi, 1.0);
        // 修正後のworld->currentと修正前のcurrent->covisibilityの姿勢を合わせて
        // 修正後のworld->covisibilityのSim3を求める
        const g2o::Sim3 pre_corrected_Sim3_world_to_covi = Sim3_curr_to_covi * g2o_Sim3_world_to_curr;
        pre_corrected_Sim3s_iw[covisibility] = pre_corrected_Sim3_world_to_covi;
    }

    return pre_corrected_Sim3s_iw;
}

void loop_closer::correct_covisibility_landmarks(const keyframe_Sim3_pairs_t& non_corrected_Sim3s_iw,
                                                 const keyframe_Sim3_pairs_t& pre_corrected_Sim3s_iw) const {
    // 修正前後の相対姿勢を使って3次元点を動かす
    for (const auto& keyfrm_Sim3_pair : pre_corrected_Sim3s_iw) {
        auto keyfrm_i = keyfrm_Sim3_pair.first;
        // world->keyframeのSim3(修正後)
        const auto pre_corrected_Sim3_iw = keyfrm_Sim3_pair.second;
        // keyframe->worldのSim3(修正後)
        const auto pre_corrected_Sim3_wi = pre_corrected_Sim3_iw.inverse();
        // world->keyframeのSim3(修正前)
        const auto& non_corrected_Sim3_iw = non_corrected_Sim3s_iw.at(keyfrm_i);

        const auto landmarks = keyfrm_i->get_landmarks();
        for (auto lm : landmarks) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            // 重複を避ける
            if (lm->keyfrm_id_in_loop_fusion_ == cur_keyfrm_->id_) {
                continue;
            }

            // 修正前後の相対姿勢を使って3次元点を動かす
            const Vec3_t pow_w = lm->get_pos_in_world();
            // 修正前のSim3でcamera座標に戻してから，修正後のSim3でworldへ戻す
            const Vec3_t corrected_pos_w = pre_corrected_Sim3_wi.map(non_corrected_Sim3_iw.map(pow_w));

            // 座標を更新
            lm->set_pos_in_world(corrected_pos_w);
            lm->update_normal_and_depth();

            // どのkeyframeの姿勢を元に座標を修正したかを保存しておく
            lm->keyfrm_id_in_loop_BA_ = keyfrm_i->id_;
            // 重複を避ける & loop fusionを行った際のcurrent keyframeを保存しておく
            lm->keyfrm_id_in_loop_fusion_ = cur_keyfrm_->id_;
        }
    }
}

void loop_closer::correct_covisibility_keyframes(const keyframe_Sim3_pairs_t& pre_corrected_Sim3s_iw) const {
    // 修正後の姿勢をセットする
    for (const auto& keyfrm_Sim3_pair : pre_corrected_Sim3s_iw) {
        auto keyfrm_i = keyfrm_Sim3_pair.first;
        const auto pre_corrected_Sim3_iw = keyfrm_Sim3_pair.second;

        const auto s = pre_corrected_Sim3_iw.scale();
        const Mat33_t rot_iw = pre_corrected_Sim3_iw.rotation().toRotationMatrix();
        const Vec3_t trans_iw = pre_corrected_Sim3_iw.translation() / s;
        const Mat44_t corrected_cam_pose_iw = util::converter::to_eigen_cam_pose(rot_iw, trans_iw);
        keyfrm_i->set_cam_pose(corrected_cam_pose_iw);

        keyfrm_i->update_connections();
    }
}

void loop_closer::replace_duplicated_landmarks(const std::vector<data::landmark*>& curr_assoc_lms_in_cand,
                                               const keyframe_Sim3_pairs_t& pre_corrected_Sim3s_iw) const {
    // currentとcandidateの間での3次元点の重複を解消する
    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        for (unsigned int idx = 0; idx < cur_keyfrm_->num_keypts_; ++idx) {
            // candidateで観測されている3次元点のうち，currentの特徴点と対応が取れているもの
            auto assoc_lm_in_cand = curr_assoc_lms_in_cand.at(idx);
            if (!assoc_lm_in_cand) {
                continue;
            }

            // currentで観測している3次元点
            auto assoc_lm_in_curr = cur_keyfrm_->get_landmark(idx);
            if (assoc_lm_in_curr) {
                // current側に対応している点がすでに存在する場合は
                // candidate側の3次元点で置き換える
                assoc_lm_in_curr->replace(assoc_lm_in_cand);
            }
            else {
                // current側に対応している点が存在しない場合は，
                // candidate側の点との対応を追加する
                cur_keyfrm_->add_landmark(assoc_lm_in_cand, idx);
                assoc_lm_in_cand->add_observation(cur_keyfrm_, idx);
                assoc_lm_in_cand->compute_descriptor();
            }
        }
    }

    // currentの周辺のキーフレームについても，重複を解消する
    const auto curr_assoc_lms_near_cand = loop_detector_.get_curr_assoc_lms_near_cand();
    match::fuse fuser(0.8);
    for (const auto& pre_corrected_Sim3_iw : pre_corrected_Sim3s_iw) {
        auto keyfrm = pre_corrected_Sim3_iw.first;
        const auto& g2o_Sim3_cw = pre_corrected_Sim3_iw.second;

        const Mat44_t Sim3_cw = util::converter::to_eigen_mat(g2o_Sim3_cw);

        // currentの特徴点と対応が取れている3次元点(curr_assoc_lms_near_cand_)をkeyfrmに投影して，3次元点の重複を探す
        std::vector<data::landmark*> lms_to_replace(curr_assoc_lms_near_cand.size(), nullptr);
        fuser.detect_duplication(keyfrm, Sim3_cw, curr_assoc_lms_near_cand, 4, lms_to_replace);

        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);
        for (unsigned int i = 0; i < curr_assoc_lms_near_cand.size(); ++i) {
            auto lm_to_replace = lms_to_replace.at(i);
            if (lm_to_replace) {
                lm_to_replace->replace(curr_assoc_lms_near_cand.at(i));
            }
        }
    }
}

auto loop_closer::extract_new_connections(const std::vector<data::keyframe*>& covisibilities) const
-> std::map<data::keyframe*, std::set<data::keyframe*>> {
    // covisibilities_の各キーフレームに対して，ループ対応によって生じた新たなedgeを検出する
    std::map<data::keyframe*, std::set<data::keyframe*>> new_connections;

    for (auto covisibility : covisibilities) {
        // update_connections()を実行する前なので，ループ対応する前の情報が得られる
        const auto neighbors_before_update = covisibility->get_covisibilities();

        // covisibility graphの情報を更新
        covisibility->update_connections();
        // 更新後の情報を取得する
        new_connections[covisibility] = covisibility->get_connected_keyframes();

        // covisibilityを削除
        for (const auto keyfrm_to_erase : covisibilities) {
            new_connections.at(covisibility).erase(keyfrm_to_erase);
        }
        // 更新前にもcovisibilityであったものは削除
        for (const auto keyfrm_to_erase : neighbors_before_update) {
            new_connections.at(covisibility).erase(keyfrm_to_erase);
        }

        // 以上の処理で，ループ対応によってcovisibilityと新たに接続したノードのみが残る
    }

    return new_connections;
}

void loop_closer::run_loop_BA(unsigned int lead_keyfrm_id) {
    spdlog::info("start loop bundle adjustment");

    const unsigned int idx = num_exec_loop_BA_;

    const auto global_bundle_adjuster = optimize::global_bundle_adjuster(map_db_, 10, false);
    global_bundle_adjuster.optimize(lead_keyfrm_id, &abort_loop_BA_);

    {
        std::lock_guard<std::mutex> lock1(mtx_loop_BA_);

        // BA中にcorrect loopが走っていたらreturn
        // BAが中止されていたらreturn
        if (idx != num_exec_loop_BA_ || abort_loop_BA_) {
            spdlog::info("abort loop bundle adjustment");
            loop_BA_is_running_ = false;
            abort_loop_BA_ = false;
            return;
        }

        spdlog::info("finish loop bundle adjustment");
        spdlog::info("updating map with pose propagation");
        local_mapper_->request_pause();

        while (!local_mapper_->is_paused() && !local_mapper_->is_terminated()) {
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }

        std::lock_guard<std::mutex> lock2(data::map_database::mtx_database_);

        // 原点のkeyframeからspanning treeを辿って，姿勢を修正していく
        std::list<data::keyframe*> keyfrms_to_check;
        keyfrms_to_check.push_back(map_db_->origin_keyfrm_);
        while (!keyfrms_to_check.empty()) {
            auto parent = keyfrms_to_check.front();
            const Mat44_t cam_pose_wp = parent->get_cam_pose_inv();

            const auto children = parent->get_spanning_children();
            for (auto child : children) {
                if (child->lead_keyfrm_id_in_loop_BA_ != lead_keyfrm_id) {
                    // BAの対象外だった場合このif文の中に入る
                    // 姿勢伝播で姿勢を補正する

                    // 修正前のparent->childの姿勢
                    const Mat44_t cam_pose_cp = child->get_cam_pose() * cam_pose_wp;
                    // BA修正後のchildの姿勢 = BA修正前のparent->childの姿勢 * BA修正後のkeyframeの姿勢
                    child->cam_pose_cw_after_BA_ = cam_pose_cp * parent->cam_pose_cw_after_BA_;
                    // 修正済みの印を付ける
                    child->lead_keyfrm_id_in_loop_BA_ = lead_keyfrm_id;
                }

                // チェック対象に追加する
                keyfrms_to_check.push_back(child);
            }

            // 更新前の姿勢を保存しておく(3次元点の移動のため)
            parent->cam_pose_cw_before_BA_ = parent->get_cam_pose();
            // 姿勢を更新
            parent->set_cam_pose(parent->cam_pose_cw_after_BA_);
            // 更新したのでcheck対象から外す
            keyfrms_to_check.pop_front();
        }

        const auto landmarks = map_db_->get_all_landmarks();
        for (auto lm : landmarks) {
            if (lm->will_be_erased()) {
                continue;
            }

            if (lm->lead_keyfrm_id_in_loop_BA_ == lead_keyfrm_id) {
                // BAの対象だった場合はそのまま位置を更新する
                lm->set_pos_in_world(lm->pos_w_after_global_BA_);
            }
            else {
                // BAの対象外だった場合はreference keyframeの姿勢移動に応じて3次元点を移す
                auto ref_keyfrm = lm->get_ref_keyframe();

                assert(ref_keyfrm->lead_keyfrm_id_in_loop_BA_ == lead_keyfrm_id);

                // BA補正前の姿勢を使って3次元点をカメラ座標に移す
                const Mat33_t rot_cw_before_BA = ref_keyfrm->cam_pose_cw_before_BA_.block<3, 3>(0, 0);
                const Vec3_t trans_cw_before_BA = ref_keyfrm->cam_pose_cw_before_BA_.block<3, 1>(0, 3);
                const Vec3_t pos_c = rot_cw_before_BA * lm->get_pos_in_world() + trans_cw_before_BA;

                // BA補正後の姿勢を使って3次元点をワールド座標に戻す
                const Mat44_t cam_pose_wc = ref_keyfrm->get_cam_pose_inv();
                const Mat33_t rot_wc = cam_pose_wc.block<3, 3>(0, 0);
                const Vec3_t trans_wc = cam_pose_wc.block<3, 1>(0, 3);
                lm->set_pos_in_world(rot_wc * pos_c + trans_wc);
            }
        }

        local_mapper_->resume();
        loop_BA_is_running_ = false;

        spdlog::info("updated map");
    }
}

void loop_closer::request_reset() {
    {
        std::lock_guard<std::mutex> lock(mtx_reset_);
        reset_is_requested_ = true;
    }

    // resetされるまで(reset_is_requested_フラグが落とされるまで)待機する
    while (true) {
        {
            std::lock_guard<std::mutex> lock(mtx_reset_);
            if (!reset_is_requested_) {
                spdlog::info("reset loop closer");
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(3000));
    }
}

void loop_closer::check_and_execute_reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    if (!reset_is_requested_) {
        return;
    }
    // リセットフラグが立っていたらリセットする
    keyfrms_queue_.clear();
    loop_detector_.set_prev_loop_correct_keyfrm_id(0);
    reset_is_requested_ = false;
}

void loop_closer::request_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
}

bool loop_closer::pause_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_;
}

bool loop_closer::is_paused() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

bool loop_closer::check_and_execute_pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    if (pause_is_requested_) {
        is_paused_ = true;
        spdlog::info("pause loop closer");
        return true;
    }
    else {
        return false;
    }
}

void loop_closer::resume() {
    // pauseとterminateのフラグを触るので排他制御
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    std::lock_guard<std::mutex> lock2(mtx_terminate_);

    // 既にloop closerが停止している場合は再開できない
    if (is_terminated_) {
        return;
    }

    // 再開する処理を行う
    is_paused_ = false;
    pause_is_requested_ = false;

    spdlog::info("resume loop closer");
}

void loop_closer::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool loop_closer::is_terminated() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool loop_closer::check_terminate() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void loop_closer::terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    is_terminated_ = true;
}

bool loop_closer::loop_BA_is_running() const {
    std::lock_guard<std::mutex> lock(mtx_loop_BA_);
    return loop_BA_is_running_;
}

void loop_closer::abort_loop_BA() {
    std::lock_guard<std::mutex> lock(mtx_loop_BA_);
    abort_loop_BA_ = true;
}

} // namespace map
} // namespace openvslam
