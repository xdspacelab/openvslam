#include "openvslam/mapping_module.h"
#include "openvslam/global_optimization_module.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/match/fuse.h"
#include "openvslam/util/converter.h"

#include <spdlog/spdlog.h>

namespace openvslam {

global_optimization_module::global_optimization_module(data::map_database* map_db, data::bow_database* bow_db,
                                                       data::bow_vocabulary* bow_vocab, const bool fix_scale)
    : loop_detector_(new module::loop_detector(bow_db, bow_vocab, fix_scale)),
      loop_bundle_adjuster_(new module::loop_bundle_adjuster(map_db)),
      graph_optimizer_(new optimize::graph_optimizer(map_db, fix_scale)) {
    spdlog::debug("CONSTRUCT: global_optimization_module");
}

global_optimization_module::~global_optimization_module() {
    abort_loop_BA();
    if (thread_for_loop_BA_) {
        thread_for_loop_BA_->join();
    }
    spdlog::debug("DESTRUCT: global_optimization_module");
}

void global_optimization_module::set_tracking_module(tracking_module* tracker) {
    tracker_ = tracker;
}

void global_optimization_module::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
    loop_bundle_adjuster_->set_mapping_module(mapper);
}

void global_optimization_module::enable_loop_detector() {
    spdlog::info("enable loop detector");
    loop_detector_->enable_loop_detector();
}

void global_optimization_module::disable_loop_detector() {
    spdlog::info("disable loop detector");
    loop_detector_->disable_loop_detector();
}

bool global_optimization_module::loop_detector_is_enabled() const {
    return loop_detector_->is_enabled();
}

void global_optimization_module::run() {
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
        loop_detector_->set_current_keyframe(cur_keyfrm_);

        // detect some loop candidate with BoW
        if (!loop_detector_->detect_loop_candidates()) {
            // could not find
            // allow the removal of the current keyframe
            cur_keyfrm_->set_to_be_erased();
            continue;
        }

        // validate candidates and select ONE candidate from them
        if (!loop_detector_->validate_candidates()) {
            // could not find
            // allow the removal of the current keyframe
            cur_keyfrm_->set_to_be_erased();
            continue;
        }

        correct_loop();
    }

    spdlog::info("terminate global optimization module");
}

void global_optimization_module::queue_keyframe(const std::shared_ptr<data::keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    if (keyfrm->id_ != 0) {
        keyfrms_queue_.push_back(keyfrm);
    }
}

bool global_optimization_module::keyframe_is_queued() const {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    return (!keyfrms_queue_.empty());
}

void global_optimization_module::correct_loop() {
    auto final_candidate_keyfrm = loop_detector_->get_selected_candidate_keyframe();

    spdlog::info("detect loop: keyframe {} - keyframe {}", final_candidate_keyfrm->id_, cur_keyfrm_->id_);
    loop_bundle_adjuster_->count_loop_BA_execution();

    // 0. pre-processing

    // 0-1. stop the mapping module and the previous loop bundle adjuster

    // pause the mapping module
    mapper_->request_pause();
    // abort the previous loop bundle adjuster
    if (thread_for_loop_BA_ || loop_bundle_adjuster_->is_running()) {
        abort_loop_BA();
    }
    // wait till the mapping module pauses
    while (!mapper_->is_paused()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 0-2. update the graph

    cur_keyfrm_->graph_node_->update_connections();

    // 1. compute the Sim3 of the covisibilities of the current keyframe whose Sim3 is already estimated by the loop detector
    //    then, the covisibilities are moved to the corrected positions
    //    finally, landmarks observed in them are also moved to the correct position using the camera poses before and after camera pose correction

    // acquire the covisibilities of the current keyframe
    std::vector<std::shared_ptr<data::keyframe>> curr_neighbors = cur_keyfrm_->graph_node_->get_covisibilities();
    curr_neighbors.push_back(cur_keyfrm_);

    // Sim3 camera poses BEFORE loop correction
    module::keyframe_Sim3_pairs_t Sim3s_nw_before_correction;
    // Sim3 camera poses AFTER loop correction
    module::keyframe_Sim3_pairs_t Sim3s_nw_after_correction;

    const auto g2o_Sim3_cw_after_correction = loop_detector_->get_Sim3_world_to_current();
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

    // 2. resolve duplications of landmarks caused by loop fusion

    const auto curr_match_lms_observed_in_cand = loop_detector_->current_matched_landmarks_observed_in_candidate();
    replace_duplicated_landmarks(curr_match_lms_observed_in_cand, Sim3s_nw_after_correction);

    // 3. extract the new connections created after loop fusion

    const auto new_connections = extract_new_connections(curr_neighbors);

    // 4. pose graph optimization

    graph_optimizer_->optimize(final_candidate_keyfrm, cur_keyfrm_, Sim3s_nw_before_correction, Sim3s_nw_after_correction, new_connections);

    // add a loop edge
    final_candidate_keyfrm->graph_node_->add_loop_edge(cur_keyfrm_);
    cur_keyfrm_->graph_node_->add_loop_edge(final_candidate_keyfrm);

    // 5. launch loop BA

    while (loop_bundle_adjuster_->is_running()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    if (thread_for_loop_BA_) {
        thread_for_loop_BA_->join();
        thread_for_loop_BA_.reset(nullptr);
    }
    thread_for_loop_BA_ = std::unique_ptr<std::thread>(new std::thread(&module::loop_bundle_adjuster::optimize, loop_bundle_adjuster_.get(), cur_keyfrm_->id_));

    // 6. post-processing

    // resume the mapping module
    mapper_->resume();

    // set the loop fusion information to the loop detector
    loop_detector_->set_loop_correct_keyframe_id(cur_keyfrm_->id_);
}

module::keyframe_Sim3_pairs_t global_optimization_module::get_Sim3s_before_loop_correction(const std::vector<std::shared_ptr<data::keyframe>>& neighbors) const {
    module::keyframe_Sim3_pairs_t Sim3s_nw_before_loop_correction;

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

module::keyframe_Sim3_pairs_t global_optimization_module::get_Sim3s_after_loop_correction(const Mat44_t& cam_pose_wc_before_correction,
                                                                                          const g2o::Sim3& g2o_Sim3_cw_after_correction,
                                                                                          const std::vector<std::shared_ptr<data::keyframe>>& neighbors) const {
    module::keyframe_Sim3_pairs_t Sim3s_nw_after_loop_correction;

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

void global_optimization_module::correct_covisibility_landmarks(const module::keyframe_Sim3_pairs_t& Sim3s_nw_before_correction,
                                                                const module::keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const {
    for (const auto& t : Sim3s_nw_after_correction) {
        auto neighbor = t.first;
        // neighbor->world AFTER loop correction
        const auto Sim3_wn_after_correction = t.second.inverse();
        // world->neighbor BEFORE loop correction
        const auto& Sim3_nw_before_correction = Sim3s_nw_before_correction.at(neighbor);

        const auto ngh_landmarks = neighbor->get_landmarks();
        for (const auto& lm : ngh_landmarks) {
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

void global_optimization_module::correct_covisibility_keyframes(const module::keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const {
    for (const auto& t : Sim3s_nw_after_correction) {
        auto neighbor = t.first;
        const auto Sim3_nw_after_correction = t.second;

        const auto s_nw = Sim3_nw_after_correction.scale();
        const Mat33_t rot_nw = Sim3_nw_after_correction.rotation().toRotationMatrix();
        const Vec3_t trans_nw = Sim3_nw_after_correction.translation() / s_nw;
        const Mat44_t cam_pose_nw = util::converter::to_eigen_cam_pose(rot_nw, trans_nw);
        neighbor->set_cam_pose(cam_pose_nw);

        // update graph
        neighbor->graph_node_->update_connections();
    }
}

void global_optimization_module::replace_duplicated_landmarks(const std::vector<std::shared_ptr<data::landmark>>& curr_match_lms_observed_in_cand,
                                                              const module::keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const {
    // resolve duplications of landmarks between the current keyframe and the loop candidate
    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        for (unsigned int idx = 0; idx < cur_keyfrm_->num_keypts_; ++idx) {
            auto curr_match_lm_in_cand = curr_match_lms_observed_in_cand.at(idx);
            if (!curr_match_lm_in_cand) {
                continue;
            }

            const auto& lm_in_curr = cur_keyfrm_->get_landmark(idx);
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
    auto curr_match_lms_observed_in_cand_covis = loop_detector_->current_matched_landmarks_observed_in_candidate_covisibilities();
    match::fuse fuser(0.8);
    for (const auto& t : Sim3s_nw_after_correction) {
        auto neighbor = t.first;
        const Mat44_t Sim3_nw_after_correction = util::converter::to_eigen_mat(t.second);

        // reproject the landmarks observed in the current keyframe to the neighbor,
        // then search duplication of the landmarks
        std::vector<std::shared_ptr<data::landmark>> lms_to_replace(curr_match_lms_observed_in_cand_covis.size(), nullptr);
        fuser.detect_duplication(neighbor, Sim3_nw_after_correction, curr_match_lms_observed_in_cand_covis, 4, lms_to_replace);

        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);
        // if any landmark duplication is found, replace it
        for (unsigned int i = 0; i < curr_match_lms_observed_in_cand_covis.size(); ++i) {
            const auto& lm_to_replace = lms_to_replace.at(i);
            if (lm_to_replace) {
                lm_to_replace->replace(curr_match_lms_observed_in_cand_covis.at(i));
            }
        }
    }
}

auto global_optimization_module::extract_new_connections(const std::vector<std::shared_ptr<data::keyframe>>& covisibilities) const
    -> std::map<std::shared_ptr<data::keyframe>, std::set<std::shared_ptr<data::keyframe>>> {
    std::map<std::shared_ptr<data::keyframe>, std::set<std::shared_ptr<data::keyframe>>> new_connections;

    for (auto covisibility : covisibilities) {
        // acquire neighbors BEFORE loop fusion (because update_connections() is not called yet)
        const auto neighbors_before_update = covisibility->graph_node_->get_covisibilities();

        // call update_connections()
        covisibility->graph_node_->update_connections();
        // acquire neighbors AFTER loop fusion
        new_connections[covisibility] = covisibility->graph_node_->get_connected_keyframes();

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

void global_optimization_module::request_reset() {
    {
        std::lock_guard<std::mutex> lock(mtx_reset_);
        reset_is_requested_ = true;
    }

    // BLOCK until reset
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

bool global_optimization_module::reset_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    return reset_is_requested_;
}

void global_optimization_module::reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    spdlog::info("reset global optimization module");
    keyfrms_queue_.clear();
    loop_detector_->set_loop_correct_keyframe_id(0);
    reset_is_requested_ = false;
}

void global_optimization_module::request_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
}

bool global_optimization_module::pause_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_;
}

bool global_optimization_module::is_paused() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

void global_optimization_module::pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    spdlog::info("pause global optimization module");
    is_paused_ = true;
}

void global_optimization_module::resume() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    std::lock_guard<std::mutex> lock2(mtx_terminate_);

    // if it has been already terminated, cannot resume
    if (is_terminated_) {
        return;
    }

    is_paused_ = false;
    pause_is_requested_ = false;

    spdlog::info("resume global optimization module");
}

void global_optimization_module::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool global_optimization_module::is_terminated() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool global_optimization_module::terminate_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void global_optimization_module::terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    is_terminated_ = true;
}

bool global_optimization_module::loop_BA_is_running() const {
    return loop_bundle_adjuster_->is_running();
}

void global_optimization_module::abort_loop_BA() {
    loop_bundle_adjuster_->abort();
}

} // namespace openvslam
