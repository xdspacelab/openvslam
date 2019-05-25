#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/data/map_database.h"
#include "openvslam/map/local_mapper.h"
#include "openvslam/map/loop_closer.h"
#include "openvslam/match/fuse.h"
#include "openvslam/optimize/global_bundle_adjuster.h"
#include "openvslam/util/converter.h"

#include <mutex>
#include <thread>

#include <spdlog/spdlog.h>

namespace openvslam {
namespace map {

loop_closer::loop_closer(data::map_database* map_db, data::bow_database* bow_db, data::bow_vocabulary* bow_vocab, const bool fix_scale)
        : loop_detector_(bow_db, bow_vocab, fix_scale), map_db_(map_db), graph_optimizer_(map_db, fix_scale) {
    spdlog::debug("CONSTRUCT: map::loop_closer");
}

loop_closer::~loop_closer() {
    abort_loop_BA();
    if (thread_for_loop_BA_) {
        thread_for_loop_BA_->join();
    }
    spdlog::debug("DESTRUCT: map::loop_closer");
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
        loop_detector_.enable_loop_detector();
    }
    else {
        spdlog::info("disable loop detector");
        loop_detector_.disable_loop_detector();
    }
}

bool loop_closer::get_loop_detector_status() const {
    return loop_detector_.is_enabled();
}

void loop_closer::run() {
    spdlog::info("start global optimization module");

    is_terminated_ = false;

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // check if termination is requested
        if (terminate_is_requested()) {
            // terminate and break
            terminate();
            break;
        }

        // check if pause is requested
        if (pause_is_requested()) {
            // pause and wait
            pause();
            // check if termination or reset is requested during pause
            while (is_paused() && !terminate_is_requested() && !reset_is_requested()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        }

        // check if reset is requested
        if (reset_is_requested()) {
            // reset and continue
            reset();
            continue;
        }

        // if the queue is empty, the following process is not needed
        if (!keyframe_is_queued()) {
            continue;
        }

        // dequeue the keyframe from the queue -> cur_keyfrm_
        {
            std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
            cur_keyfrm_ = keyfrms_queue_.front();
            keyfrms_queue_.pop_front();
        }

        // not to be removed during loop detection and correction
        cur_keyfrm_->set_not_to_be_erased();

        // pass the current keyframe to the loop detector
        loop_detector_.set_current_keyframe(cur_keyfrm_);

        // detect some loop candidate with BoW
        if (!loop_detector_.detect_loop_candidates()) {
            // could not find
            // allow the removal of the current keyframe
            cur_keyfrm_->set_to_be_erased();
            continue;
        }

        // validate candidates and select ONE candidate from them
        if (!loop_detector_.validate_candidates()) {
            // could not find
            // allow the removal of the current keyframe
            cur_keyfrm_->set_to_be_erased();
            continue;
        }

        correct_loop();
    }

    spdlog::info("terminate global optimization module");
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
    auto final_candidate_keyfrm = loop_detector_.get_selected_candidate_keyframe();

    spdlog::info("detect loop: keyframe {} - keyframe {}", final_candidate_keyfrm->id_, cur_keyfrm_->id_);
    ++num_exec_loop_BA_;

    // 0. pre-processing

    // 0-1. stop the mapping module and the previous loop bundle adjuster

    // pause the mapping module
    local_mapper_->request_pause();
    // abort the previous loop bundle adjuster
    if (loop_BA_is_running() || thread_for_loop_BA_) {
        abort_loop_BA();
    }
    // wait till the mapping module pauses
    while (!local_mapper_->is_paused()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 0-2. update the graph

    cur_keyfrm_->update_connections();

    // 1. compute the Sim3 of the covisibilities of the current keyframe whose Sim3 is already estimated by the loop detector
    //    then, the covisibilities are moved to the corrected positions
    //    finally, landmarks observed in them are also moved to the correct position using the camera poses before and after camera pose correction

    // acquire the covisibilities of the current keyframe
    std::vector<data::keyframe*> curr_neighbors = cur_keyfrm_->get_covisibilities();
    curr_neighbors.push_back(cur_keyfrm_);

    // Sim3 camera poses BEFORE loop correction
    keyframe_Sim3_pairs_t Sim3s_nw_before_correction;
    // Sim3 camera poses AFTER loop correction
    keyframe_Sim3_pairs_t Sim3s_nw_after_correction;

    const auto g2o_Sim3_cw_after_correction = loop_detector_.get_Sim3_world_to_current();
    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        // camera pose of the current keyframe BEFORE loop correction
        const Mat44_t cam_pose_wc_before_correction = cur_keyfrm_->get_cam_pose_inv();

        // compute Sim3s BEFORE loop correction
        Sim3s_nw_before_correction = get_Sim3s_before_loop_correction(curr_neighbors);
        // compute Sim3s AFTER loop correction
        Sim3s_nw_after_correction = get_Sim3s_after_loop_correction(cam_pose_wc_before_correction, g2o_Sim3_cw_after_correction, curr_neighbors);

        // correct covibisibility landmark positions
        correct_covisibility_landmarks(Sim3s_nw_before_correction, Sim3s_nw_after_correction);
        // correct covisibility keyframe camera poses
        correct_covisibility_keyframes(Sim3s_nw_after_correction);
    }

    // 2. resolve duplications of landmarks after loop fusion

    const auto curr_match_lms_observed_in_cand = loop_detector_.current_matched_landmarks_observed_in_candidate();
    replace_duplicated_landmarks(curr_match_lms_observed_in_cand, Sim3s_nw_after_correction);

    // 3. extract the new connections created after loop fusion

    const auto new_connections = extract_new_connections(curr_neighbors);

    // 4. pose graph optimization

    graph_optimizer_.optimize(final_candidate_keyfrm, cur_keyfrm_, Sim3s_nw_before_correction, Sim3s_nw_after_correction, new_connections);

    // add a loop edge
    final_candidate_keyfrm->add_loop_edge(cur_keyfrm_);
    cur_keyfrm_->add_loop_edge(final_candidate_keyfrm);

    // 5. launch loop BA

    {
        std::lock_guard<std::mutex> lock1(mtx_loop_BA_);
        if (thread_for_loop_BA_) {
            thread_for_loop_BA_->join();
            thread_for_loop_BA_.reset(nullptr);
        }
        loop_BA_is_running_ = true;
        abort_loop_BA_ = false;
        thread_for_loop_BA_ = std::unique_ptr<std::thread>(new std::thread(&loop_closer::run_loop_BA, this, cur_keyfrm_->id_));
    }

    // 6. post-processing

    // resume the mapping module
    local_mapper_->resume();

    // set the loop fusion information to the loop detector
    loop_detector_.set_loop_correct_keyframe_id(cur_keyfrm_->id_);
}

keyframe_Sim3_pairs_t loop_closer::get_Sim3s_before_loop_correction(const std::vector<data::keyframe*>& neighbors) const {
    keyframe_Sim3_pairs_t Sim3s_nw_before_loop_correction;

    for (const auto neighbor : neighbors) {
        // camera pose of `neighbor` BEFORE loop correction
        const Mat44_t cam_pose_nw = neighbor->get_cam_pose();
        // create Sim3 from SE3
        const Mat33_t& rot_nw = cam_pose_nw.block<3, 3>(0, 0);
        const Vec3_t& trans_nw = cam_pose_nw.block<3, 1>(0, 3);
        const g2o::Sim3 Sim3_nw_before_correction(rot_nw, trans_nw, 1.0);
        Sim3s_nw_before_loop_correction[neighbor] = Sim3_nw_before_correction;
    }

    return Sim3s_nw_before_loop_correction;
}

keyframe_Sim3_pairs_t loop_closer::get_Sim3s_after_loop_correction(const Mat44_t& cam_pose_wc_before_correction,
                                                                   const g2o::Sim3& g2o_Sim3_cw_after_correction,
                                                                   const std::vector<data::keyframe*>& neighbors) const {
    keyframe_Sim3_pairs_t Sim3s_nw_after_loop_correction;

    for (auto neighbor : neighbors) {
        // camera pose of `neighbor` BEFORE loop correction
        const Mat44_t cam_pose_nw_before_correction = neighbor->get_cam_pose();
        // create the relative Sim3 from the current to `neighbor`
        const Mat44_t cam_pose_nc = cam_pose_nw_before_correction * cam_pose_wc_before_correction;
        const Mat33_t& rot_nc = cam_pose_nc.block<3, 3>(0, 0);
        const Vec3_t& trans_nc = cam_pose_nc.block<3, 1>(0, 3);
        const g2o::Sim3 Sim3_nc(rot_nc, trans_nc, 1.0);
        // compute the camera poses AFTER loop correction of the neighbors
        const g2o::Sim3 Sim3_nw_after_correction = Sim3_nc * g2o_Sim3_cw_after_correction;
        Sim3s_nw_after_loop_correction[neighbor] = Sim3_nw_after_correction;
    }

    return Sim3s_nw_after_loop_correction;
}

void loop_closer::correct_covisibility_landmarks(const keyframe_Sim3_pairs_t& Sim3s_nw_before_correction,
                                                 const keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const {
    for (const auto& t : Sim3s_nw_after_correction) {
        auto neighbor = t.first;
        // neighbor->world AFTER loop correction
        const auto Sim3_wn_after_correction = t.second.inverse();
        // world->neighbor BEFORE loop correction
        const auto& Sim3_nw_before_correction = Sim3s_nw_before_correction.at(neighbor);

        const auto ngh_landmarks = neighbor->get_landmarks();
        for (auto lm : ngh_landmarks) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            // avoid duplication
            if (lm->loop_fusion_identifier_ == cur_keyfrm_->id_) {
                continue;
            }
            lm->loop_fusion_identifier_ = cur_keyfrm_->id_;

            // correct position of `lm`
            const Vec3_t pos_w_before_correction = lm->get_pos_in_world();
            const Vec3_t pos_w_after_correction = Sim3_wn_after_correction.map(Sim3_nw_before_correction.map(pos_w_before_correction));
            lm->set_pos_in_world(pos_w_after_correction);
            // update geometry
            lm->update_normal_and_depth();

            // record the reference keyframe used in loop fusion of landmarks
            lm->ref_keyfrm_id_in_loop_fusion_ = neighbor->id_;
        }
    }
}

void loop_closer::correct_covisibility_keyframes(const keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const {
    for (const auto& t : Sim3s_nw_after_correction) {
        auto neighbor = t.first;
        const auto Sim3_nw_after_correction = t.second;

        const auto s_nw = Sim3_nw_after_correction.scale();
        const Mat33_t rot_nw = Sim3_nw_after_correction.rotation().toRotationMatrix();
        const Vec3_t trans_nw = Sim3_nw_after_correction.translation() / s_nw;
        const Mat44_t cam_pose_nw = util::converter::to_eigen_cam_pose(rot_nw, trans_nw);
        neighbor->set_cam_pose(cam_pose_nw);

        // update graph
        neighbor->update_connections();
    }
}

void loop_closer::replace_duplicated_landmarks(const std::vector<data::landmark*>& curr_match_lms_observed_in_cand,
                                               const keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const {
    // resolve duplications of landmarks between the current keyframe and the loop candidate
    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        for (unsigned int idx = 0; idx < cur_keyfrm_->num_keypts_; ++idx) {
            auto curr_match_lm_in_cand = curr_match_lms_observed_in_cand.at(idx);
            if (!curr_match_lm_in_cand) {
                continue;
            }

            auto lm_in_curr = cur_keyfrm_->get_landmark(idx);
            if (lm_in_curr) {
                // if the landmark corresponding `idx` exists,
                // replace it with `curr_match_lm_in_cand` (observed in the candidate)
                lm_in_curr->replace(curr_match_lm_in_cand);
            }
            else {
                // if landmark corresponding `idx` does not exists,
                // add association between the current keyframe and `curr_match_lm_in_cand`
                cur_keyfrm_->add_landmark(curr_match_lm_in_cand, idx);
                curr_match_lm_in_cand->add_observation(cur_keyfrm_, idx);
                curr_match_lm_in_cand->compute_descriptor();
            }
        }
    }

    // resolve duplications of landmarks between the current keyframe and the candidates of the loop candidate
    const auto curr_match_lms_observed_in_cand_covis = loop_detector_.current_matched_landmarks_observed_in_candidate_covisibilities();
    match::fuse fuser(0.8);
    for (const auto& t : Sim3s_nw_after_correction) {
        auto neighbor = t.first;
        const Mat44_t Sim3_nw_after_correction = util::converter::to_eigen_mat(t.second);

        // reproject the landmarks observed in the current keyframe to the neighbor,
        // then search duplication of the landmarks
        std::vector<data::landmark*> lms_to_replace(curr_match_lms_observed_in_cand_covis.size(), nullptr);
        fuser.detect_duplication(neighbor, Sim3_nw_after_correction, curr_match_lms_observed_in_cand_covis, 4, lms_to_replace);

        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);
        // if any landmark duplication is found, replace it
        for (unsigned int i = 0; i < curr_match_lms_observed_in_cand_covis.size(); ++i) {
            auto lm_to_replace = lms_to_replace.at(i);
            if (lm_to_replace) {
                lm_to_replace->replace(curr_match_lms_observed_in_cand_covis.at(i));
            }
        }
    }
}

auto loop_closer::extract_new_connections(const std::vector<data::keyframe*>& covisibilities) const
-> std::map<data::keyframe*, std::set<data::keyframe*>> {
    std::map<data::keyframe*, std::set<data::keyframe*>> new_connections;

    for (auto covisibility : covisibilities) {
        // acquire neighbors BEFORE loop fusion (because update_connections() is not called yet)
        const auto neighbors_before_update = covisibility->get_covisibilities();

        // call update_connections()
        covisibility->update_connections();
        // acquire neighbors AFTER loop fusion
        new_connections[covisibility] = covisibility->get_connected_keyframes();

        // remove covisibilities
        for (const auto keyfrm_to_erase : covisibilities) {
            new_connections.at(covisibility).erase(keyfrm_to_erase);
        }
        // remove nighbors before loop fusion
        for (const auto keyfrm_to_erase : neighbors_before_update) {
            new_connections.at(covisibility).erase(keyfrm_to_erase);
        }
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
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(3000));
    }
}

bool loop_closer::reset_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    return reset_is_requested_;
}

void loop_closer::reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    spdlog::info("reset global optimization module");
    keyfrms_queue_.clear();
    loop_detector_.set_loop_correct_keyframe_id(0);
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

void loop_closer::pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    spdlog::info("pause global optimization module");
    is_paused_ = true;
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

    spdlog::info("resume global optimization module");
}

void loop_closer::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool loop_closer::is_terminated() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool loop_closer::terminate_is_requested() const {
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
