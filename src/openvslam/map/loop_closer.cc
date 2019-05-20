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
        : map_db_(map_db), bow_db_(bow_db), bow_vocab_(bow_vocab),
          graph_optimizer_(map_db, fix_scale), transform_optimizer_(fix_scale), fix_scale_(fix_scale) {
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
            if (detect_loop_candidate()) {
                if (validate_candidates()) {
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

bool loop_closer::detect_loop_candidate() {
    {
        std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
        cur_keyfrm_ = keyfrms_queue_.front();
        keyfrms_queue_.pop_front();
        // ループ探索中はこのキーフレームの削除を禁止する
        cur_keyfrm_->set_not_to_be_erased();
    }

    // loop detectorが停止されていたらループ検出を行わない
    // or ループ修正を行ってからすぐはループ修正を行わない
    if (!loop_detector_is_enabled_ || cur_keyfrm_->id_ < prev_loop_correct_keyfrm_id_ + 10) {
        // このキーフレームでは何もしないのでデータベースに追加する
        bow_db_->add_keyframe(cur_keyfrm_);
        // 削除を許可
        cur_keyfrm_->set_to_be_erased();
        return false;
    }

    // 1. BoWデータベースに問い合わせてループ候補を探す

    // 1-1. 近傍キーフレームで，BoWの類似度の最小値を計算する -> 探索の際の最小スコアにする

    const float min_score = compute_min_score_in_covisibilities(cur_keyfrm_);

    // 1-2. min_scoreを使ってBoWデータベースに類似キーフレームを問い合わせる

    const auto init_loop_candidates = bow_db_->acquire_loop_candidates(cur_keyfrm_, min_score);

    // 1-3. 候補がなかったらループ修正を行わない

    if (init_loop_candidates.empty()) {
        // このキーフレームでは何もしないのでデータベースに追加する
        bow_db_->add_keyframe(cur_keyfrm_);
        // 連続してキーフレーム集合が検出されなかったのでバッファをclearする
        cont_detected_keyfrm_sets_.clear();
        // 削除を許可
        cur_keyfrm_->set_to_be_erased();
        return false;
    }

    // 2. init_loop_candidatesの各ループ候補を，単一フレームから近傍キーフレームに拡げる
    //    各キーフレーム集合が何回連続して検出されたかを記録していく
    //    (前回同じキーフレーム集合が検出されていれば，cont_detected_keyfrm_sets_に一致するキーフレーム集合が存在する)
    //    (キーフレーム集合が一致している = 2つの集合の積集合が空集合でないこと)

    const auto curr_cont_detected_keyfrm_sets =
            find_continuously_detected_keyframe_sets(cont_detected_keyfrm_sets_, init_loop_candidates);

    // 3. 閾値(min_continuity_)以上連続して検出されたキーフレーム集合を新たなループ候補として採用する

    loop_candidates_to_validate_.clear();

    for (auto& curr : curr_cont_detected_keyfrm_sets) {
        const auto candidate_keyfrm = curr.lead_keyfrm_;
        const auto continuity = curr.continuity_;
        if (min_continuity_ <= continuity) {
            loop_candidates_to_validate_.push_back(candidate_keyfrm);
        }
    }

    // 4. 次回に備えてメンバ変数を更新

    // 連続してループ検出されたキーフレーム集合と連続回数を更新
    cont_detected_keyfrm_sets_ = curr_cont_detected_keyfrm_sets;

    // ループ検出が終了したのでBoWデータベースに追加
    bow_db_->add_keyframe(cur_keyfrm_);

    if (loop_candidates_to_validate_.empty()) {
        // ループ候補がなければ現在のキーフレームの削除を許可する
        cur_keyfrm_->set_to_be_erased();
        return false;
    }
    else {
        // 引き続きループ修正を行うので，現在のキーフレームの削除は禁止
        return true;
    }
}

float loop_closer::compute_min_score_in_covisibilities(data::keyframe* keyfrm) const {
    // スコアの最大値は1.0
    float min_score = 1.0;

    // キーフレームとそのcovisibilityの間で類似度を計算して，最小値を探索する
    const auto covisibilities = keyfrm->get_covisibilities();
    const auto& bow_vec_1 = keyfrm->bow_vec_;
    for (const auto covisibility : covisibilities) {
        if (covisibility->will_be_erased()) {
            continue;
        }
        const auto& bow_vec_2 = covisibility->bow_vec_;
        // currentとcovisibilityの間で類似度を計算して，最小値を探索する
#ifdef USE_DBOW2
        const float score = bow_vocab_->score(bow_vec_1, bow_vec_2);
#else
        const float score = fbow::BoWVector::score(bow_vec_1, bow_vec_2);
#endif
        if (score < min_score) {
            min_score = score;
        }
    }

    return min_score;
}

keyframe_sets loop_closer::find_continuously_detected_keyframe_sets(const keyframe_sets& prev_cont_detected_keyfrm_sets,
                                                                    const std::vector<data::keyframe*>& keyfrms_to_search) const {
    // 各キーフレーム集合が何回連続して検出されたかを数える

    // 連続してループ検出されたキーフレーム集合と回数を保存しておく
    std::vector<keyframe_set> curr_cont_detected_keyfrm_sets;

    // 集合同士が全単射になるように制限をかけるためのフラグ集合
    // prev_cont_detected_keyfrm_setsの各キーフレーム集合に対応するキーフレーム集合がただ1つになるようにする
    std::map<std::set<data::keyframe*>, bool> already_matched;
    for (const auto& prev : prev_cont_detected_keyfrm_sets) {
        already_matched[prev.keyfrm_set_] = false;
    }
    // これ以降はalready_matched.at()でアクセスする(constant access)

    for (const auto& loop_candidate : keyfrms_to_search) {
        // 各ループ候補を，単一フレームから近傍キーフレーム集合に拡げる -> curr_keyfrm_set
        auto curr_keyfrm_set = loop_candidate->get_connected_keyframes();

        // first_candidate_setがいずれとも一致しなかったら初期化するためにフラグを設定しておく
        bool initialization_is_needed = true;

        // 前回のループ検出までに連続して検出されているキーフレーム集合と比較して，一致するか確認
        for (const auto& prev : prev_cont_detected_keyfrm_sets) {
            // prev.keyfrm_set_: キーフレームの集合
            // prev.lead_keyfrm_: 中心のキーフレーム
            // prev.continuity_: 連続検出回数

            // 前回ループ候補として検出されたキーフレーム集合(prev.keyfrm_set_)が
            // 既に今回ループ候補として検出されたキーフレーム集合のうちどれかと一致するとされていたらスルーする
            if (already_matched.at(prev.keyfrm_set_)) {
                continue;
            }

            // cand_keyfrm_setとprev.keyfrm_set_の
            // 積集合が空集合でなければ一致しているとする
            if (prev.intersection_is_empty(curr_keyfrm_set)) {
                continue;
            }

            // curr_keyfrm_setがprev_cont_detected_keyfrm_setsのどれかと一致したので，
            // curr_keyfrm_setを新たに追加する必要はない
            initialization_is_needed = false;

            // curr_keyfrm_setが連続してcurr_continuity回検出された -> 集計
            const auto curr_continuity = prev.continuity_ + 1;
            curr_cont_detected_keyfrm_sets.emplace_back(
                    keyframe_set{curr_keyfrm_set, loop_candidate, curr_continuity});

            // cont_detected_keyfrm_setがprev.keyfrm_set_と一致したのでフラグを立てる
            already_matched.at(prev.keyfrm_set_) = true;
        }

        // 前回のキーフレーム集合に対し，curr_keyfrm_setがどれとも一致しなかったらそれを新たに追加しておく
        if (initialization_is_needed) {
            curr_cont_detected_keyfrm_sets.emplace_back(
                    keyframe_set{curr_keyfrm_set, loop_candidate, 0});
        }
    }

    return curr_cont_detected_keyfrm_sets;
}

bool loop_closer::validate_candidates() {
    // ループ候補の削除を禁止する
    for (const auto candidate : loop_candidates_to_validate_) {
        candidate->set_not_to_be_erased();
    }

    // 1. ループ候補の各キーフレームに対し，観測されている3次元点を使って相似変換行列を求め，Sim3の条件を満たされているものを探す

    const bool candidate_is_found = select_loop_candidate_via_Sim3(loop_candidates_to_validate_, final_candidate_keyfrm_,
                                                                   g2o_Sim3_world_to_curr_, curr_assoc_lms_in_cand_);
    Sim3_world_to_curr_ = util::converter::to_eigen_mat(g2o_Sim3_world_to_curr_);

    if (!candidate_is_found) {
        for (auto& loop_candidate : loop_candidates_to_validate_) {
            loop_candidate->set_to_be_erased();
        }
        cur_keyfrm_->set_to_be_erased();
        return false;
    }

    spdlog::debug("detect loop candidate via Sim3 estimation: keyframe {} - keyframe {}", final_candidate_keyfrm_->id_, cur_keyfrm_->id_);

    // 2. 得られた相似変換行列を使って，ループ候補周辺の3次元点をcurrentのキーフレームに再投影し，
    //    さらに2D-3D対応を追加する

    // candidateとcandidate周辺で見えている3次元点と，currentの特徴点との対応情報
    curr_assoc_lms_near_cand_.clear();

    auto cand_covisibilities = final_candidate_keyfrm_->get_covisibilities();
    cand_covisibilities.push_back(final_candidate_keyfrm_);

    for (const auto covisibility : cand_covisibilities) {
        const auto lms_in_covisibility = covisibility->get_landmarks();
        for (const auto lm : lms_in_covisibility) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            if (lm->keyfrm_id_in_loop_validation_ == cur_keyfrm_->id_) {
                continue;
            }
            curr_assoc_lms_near_cand_.push_back(lm);
            lm->keyfrm_id_in_loop_validation_ = cur_keyfrm_->id_;
        }
    }

    // candidate周辺の3次元点(curr_assoc_lms_near_cand_)をSim3(Sim3_world_to_curr_)を用いて
    // current keyframeに再投影し，2次元点-3次元点の対応(curr_assoc_lms_in_cand_)を追加する
    // curr_assoc_lms_in_cand_ですでに対応が取られている3次元点は再投影する必要が無いので除外する
    match::projection projection_matcher(0.75, true);
    projection_matcher.match_by_Sim3_transform(cur_keyfrm_, Sim3_world_to_curr_, curr_assoc_lms_near_cand_, curr_assoc_lms_in_cand_, 10);

    // 最終的なマッチング数を数える
    unsigned int num_final_matches = 0;
    for (const auto curr_assoc_lm_in_cand : curr_assoc_lms_in_cand_) {
        if (curr_assoc_lm_in_cand) {
            ++num_final_matches;
        }
    }

    spdlog::debug("acquired {} matches after projection-match", num_final_matches);

    constexpr unsigned int num_final_matches_thr = 40;
    if (num_final_matches_thr <= num_final_matches) {
        // loop-closeを実行する
        for (const auto loop_candidate : loop_candidates_to_validate_) {
            // 最終的なループ候補以外の削除を許可する
            if (*loop_candidate == *final_candidate_keyfrm_) {
                continue;
            }
            loop_candidate->set_to_be_erased();
        }
        return true;
    }
    else {
        // loop-closeを実行しない
        spdlog::debug("destruct loop candidate because enough matches not acquired (< {})", num_final_matches_thr);
        // ループ候補全ての削除を許可する
        for (const auto loop_candidate : loop_candidates_to_validate_) {
            loop_candidate->set_to_be_erased();
        }
        cur_keyfrm_->set_to_be_erased();
        return false;
    }
}

bool loop_closer::select_loop_candidate_via_Sim3(const std::vector<data::keyframe*>& loop_candidates,
                                                 data::keyframe*& selected_candidate,
                                                 g2o::Sim3& g2o_Sim3_world_to_curr,
                                                 std::vector<data::landmark*>& curr_assoc_lms_in_cand) const {
    // ループ候補の各キーフレームに対し，観測されている3次元点を使って相似変換行列を求める
    // 相似変換行列は，線形方程式のRANSAC -> 非線形最適化の順に推定し，
    // 都度，インライア数が閾値を下回っていたらそのループ候補は破棄する

    match::bow_tree bow_matcher(0.75, true);
    match::projection projection_matcher(0.75, true);

    for (const auto candidate : loop_candidates) {
        // 削除対象であれば破棄
        if (candidate->will_be_erased()) {
            continue;
        }

        // BoWツリーで対応点を探索する
        // TODO: BF-matchに切り替える
        // cur_keyfrm_とcandidateの間で対応点を探索し，candidateで観測されている3次元点との対応を得る
        // (Sim3の計算に，currentで観測されている3次元点とcandidateで観測されている3次元点の対応が必要なため)
        std::vector<data::landmark*> assoc_lms_in_cand;
        const auto num_matches = bow_matcher.match_keyframes(cur_keyfrm_, candidate, assoc_lms_in_cand);
        // 対応数が閾値より少なかったら破棄
        if (num_matches < 20) {
            continue;
        }

        // 対応点を使ってSim3を求める
        // keyframe1: current keyframe, keyframe2: candidate keyframe
        // 2->1(candidate->current)のSim3を求める
        solver::sim3_solver solver(cur_keyfrm_, candidate, assoc_lms_in_cand, fix_scale_);
        solver.set_ransac_parameters(0.999, 20, 300);
        const auto estimated = solver.estimate();
        if (!estimated) {
            continue;
        }

        spdlog::debug("found loop candidate via linear Sim3 estimation: keyframe {} - keyframe {}", candidate->id_, cur_keyfrm_->id_);

        const Mat33_t rot_cand_to_curr = solver.get_best_rotation_12();
        const Vec3_t trans_cand_to_curr = solver.get_best_translation_12();
        const float scale_cand_to_curr = solver.get_best_scale_12();

        projection_matcher.match_keyframes_mutually(cur_keyfrm_, candidate, assoc_lms_in_cand, scale_cand_to_curr, rot_cand_to_curr, trans_cand_to_curr, 7.5);

        g2o::Sim3 g2o_sim3_cand_to_curr(rot_cand_to_curr, trans_cand_to_curr, scale_cand_to_curr);
        const auto num_optimized_inliers = transform_optimizer_.optimize(cur_keyfrm_, candidate, assoc_lms_in_cand, g2o_sim3_cand_to_curr, 10);

        // インライア数が閾値より少なかったら破棄
        if (num_optimized_inliers < 20) {
            continue;
        }

        spdlog::debug("found loop candidate via nonlinear Sim3 optimization: keyframe {} - keyframe {}", candidate->id_, cur_keyfrm_->id_);

        selected_candidate = candidate;
        g2o_Sim3_world_to_curr = g2o_sim3_cand_to_curr * g2o::Sim3(candidate->get_rotation(), candidate->get_translation(), 1.0);
        curr_assoc_lms_in_cand = assoc_lms_in_cand;

        return true;
    }

    return false;
}

void loop_closer::correct_loop() {
    spdlog::info("detect loop: keyframe {} - keyframe {}", final_candidate_keyfrm_->id_, cur_keyfrm_->id_);
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

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        // 修正前のcurrent keyframeの姿勢
        const Mat44_t cam_pose_curr_to_world = cur_keyfrm_->get_cam_pose_inv();

        // validate_candidates()で推定したg2o_Sim3_world_to_curr_(ループ修正後のcurrent keyframeのSim3)
        // を用いて，ループ修正前後のcurr_covisibilitiesのSim3を求める
        non_corrected_Sim3s_iw = get_non_corrected_Sim3s(curr_covisibilities);
        pre_corrected_Sim3s_iw = get_pre_corrected_Sim3s(cam_pose_curr_to_world, g2o_Sim3_world_to_curr_, curr_covisibilities);

        // 修正前後の相対姿勢を使って3次元点を動かす
        correct_covisibility_landmarks(non_corrected_Sim3s_iw, pre_corrected_Sim3s_iw);
        // 修正後の姿勢に更新する
        correct_covisibility_keyframes(pre_corrected_Sim3s_iw);
    }

    // 3次元点の重複を解消する
    replace_duplicated_landmarks(curr_assoc_lms_in_cand_, pre_corrected_Sim3s_iw);

    // curr_covisibilities_の各キーフレームに対して，ループ対応によって生じた新たなedgeを検出する
    const auto new_connections = extract_new_connections(curr_covisibilities);

    // pose graph optimization
    graph_optimizer_.optimize(final_candidate_keyfrm_, cur_keyfrm_, non_corrected_Sim3s_iw, pre_corrected_Sim3s_iw, new_connections);

    // loop edgeを追加
    final_candidate_keyfrm_->add_loop_edge(cur_keyfrm_);
    cur_keyfrm_->add_loop_edge(final_candidate_keyfrm_);

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

    prev_loop_correct_keyfrm_id_ = cur_keyfrm_->id_;
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
    match::fuse fuser(0.8);
    for (const auto& pre_corrected_Sim3_iw : pre_corrected_Sim3s_iw) {
        auto keyfrm = pre_corrected_Sim3_iw.first;
        const auto& g2o_Sim3_cw = pre_corrected_Sim3_iw.second;

        const Mat44_t Sim3_cw = util::converter::to_eigen_mat(g2o_Sim3_cw);

        // currentの特徴点と対応が取れている3次元点(curr_assoc_lms_near_cand_)をkeyfrmに投影して，3次元点の重複を探す
        std::vector<data::landmark*> lms_to_replace(curr_assoc_lms_near_cand_.size(), nullptr);
        fuser.detect_duplication(keyfrm, Sim3_cw, curr_assoc_lms_near_cand_, 4, lms_to_replace);

        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);
        for (unsigned int i = 0; i < curr_assoc_lms_near_cand_.size(); ++i) {
            auto lm_to_replace = lms_to_replace.at(i);
            if (lm_to_replace) {
                lm_to_replace->replace(curr_assoc_lms_near_cand_.at(i));
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
    prev_loop_correct_keyfrm_id_ = 0;
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
