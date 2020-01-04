#include "openvslam/type.h"
#include "openvslam/mapping_module.h"
#include "openvslam/global_optimization_module.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/match/fuse.h"
#include "openvslam/match/robust.h"
#include "openvslam/module/two_view_triangulator.h"
#include "openvslam/solve/essential_solver.h"

#include <unordered_set>
#include <thread>

#include <spdlog/spdlog.h>

namespace openvslam {

mapping_module::mapping_module(data::map_database* map_db, const bool is_monocular)
    : local_map_cleaner_(new module::local_map_cleaner(is_monocular)), map_db_(map_db),
      local_bundle_adjuster_(new optimize::local_bundle_adjuster()), is_monocular_(is_monocular) {
    spdlog::debug("CONSTRUCT: mapping_module");
}

mapping_module::~mapping_module() {
    spdlog::debug("DESTRUCT: mapping_module");
}

void mapping_module::set_tracking_module(tracking_module* tracker) {
    tracker_ = tracker;
}

void mapping_module::set_global_optimization_module(global_optimization_module* global_optimizer) {
    global_optimizer_ = global_optimizer;
}

void mapping_module::run() {
    spdlog::info("start mapping module");

    is_terminated_ = false;

    while (true) {
        // waiting time for the other threads
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // LOCK
        set_keyframe_acceptability(false);

        // check if termination is requested
        if (terminate_is_requested()) {
            // terminate and break
            terminate();
            break;
        }

        // check if pause is requested
        if (pause_is_requested()) {
            // if any keyframe is queued, all of them must be processed before the pause
            while (keyframe_is_queued()) {
                // create and extend the map with the new keyframe
                mapping_with_new_keyframe();
                // send the new keyframe to the global optimization module
                global_optimizer_->queue_keyframe(cur_keyfrm_);
            }
            // pause and wait
            pause();
            // check if termination or reset is requested during pause
            while (is_paused() && !terminate_is_requested() && !reset_is_requested()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        }

        // check if reset is requested
        if (reset_is_requested()) {
            // reset, UNLOCK and continue
            reset();
            set_keyframe_acceptability(true);
            continue;
        }

        // if the queue is empty, the following process is not needed
        if (!keyframe_is_queued()) {
            // UNLOCK and continue
            set_keyframe_acceptability(true);
            continue;
        }

        // create and extend the map with the new keyframe
        mapping_with_new_keyframe();
        // send the new keyframe to the global optimization module
        global_optimizer_->queue_keyframe(cur_keyfrm_);

        // LOCK end
        set_keyframe_acceptability(true);
    }

    spdlog::info("terminate mapping module");
}

void mapping_module::queue_keyframe(data::keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    keyfrms_queue_.push_back(keyfrm);
    abort_local_BA_ = true;
}

unsigned int mapping_module::get_num_queued_keyframes() const {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    return keyfrms_queue_.size();
}

bool mapping_module::keyframe_is_queued() const {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    return !keyfrms_queue_.empty();
}

bool mapping_module::get_keyframe_acceptability() const {
    return keyfrm_acceptability_;
}

void mapping_module::set_keyframe_acceptability(const bool acceptability) {
    keyfrm_acceptability_ = acceptability;
}

void mapping_module::abort_local_BA() {
    abort_local_BA_ = true;
}

void mapping_module::mapping_with_new_keyframe() {
    // dequeue
    {
        std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
        // dequeue -> cur_keyfrm_
        cur_keyfrm_ = keyfrms_queue_.front();
        keyfrms_queue_.pop_front();
    }

    // set the origin keyframe
    local_map_cleaner_->set_origin_keyframe_id(map_db_->origin_keyfrm_->id_);

    // store the new keyframe to the database
    store_new_keyframe();

    // remove redundant landmarks
    local_map_cleaner_->remove_redundant_landmarks(cur_keyfrm_->id_);

    // triangulate new landmarks between the current frame and each of the covisibilities
    create_new_landmarks();

    if (keyframe_is_queued()) {
        return;
    }

    // detect and resolve the duplication of the landmarks observed in the current frame
    update_new_keyframe();

    if (keyframe_is_queued() || pause_is_requested()) {
        return;
    }

    // local bundle adjustment
    abort_local_BA_ = false;
    if (2 < map_db_->get_num_keyframes()) {
        local_bundle_adjuster_->optimize(cur_keyfrm_, &abort_local_BA_);
    }
    local_map_cleaner_->remove_redundant_keyframes(cur_keyfrm_);
}

void mapping_module::store_new_keyframe() {
    // compute BoW feature vector
    cur_keyfrm_->compute_bow();

    // update graph
    const auto cur_lms = cur_keyfrm_->get_landmarks();
    for (unsigned int idx = 0; idx < cur_lms.size(); ++idx) {
        auto lm = cur_lms.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // if `lm` does not have the observation information from `cur_keyfrm_`,
        // add the association between the keyframe and the landmark
        if (lm->is_observed_in_keyframe(cur_keyfrm_)) {
            // if `lm` is correctly observed, make it be checked by the local map cleaner
            local_map_cleaner_->add_fresh_landmark(lm);
            continue;
        }

        // update connection
        lm->add_observation(cur_keyfrm_, idx);
        // update geometry
        lm->update_normal_and_depth();
        lm->compute_descriptor();
    }
    cur_keyfrm_->graph_node_->update_connections();

    // store the new keyframe to the map database
    map_db_->add_keyframe(cur_keyfrm_);
}

void mapping_module::create_new_landmarks() {
    // get the covisibilities of `cur_keyfrm_`
    // in order to triangulate landmarks between `cur_keyfrm_` and each of the covisibilities
    constexpr unsigned int num_covisibilities = 10;
    const auto cur_covisibilities = cur_keyfrm_->graph_node_->get_top_n_covisibilities(num_covisibilities * (is_monocular_ ? 2 : 1));

    // lowe's_ratio will not be used
    match::robust robust_matcher(0.0, false);

    // camera center of the current keyframe
    const Vec3_t cur_cam_center = cur_keyfrm_->get_cam_center();

    for (unsigned int i = 0; i < cur_covisibilities.size(); ++i) {
        // if any keyframe is queued, abort the triangulation
        if (1 < i && keyframe_is_queued()) {
            return;
        }

        // get the neighbor keyframe
        auto ngh_keyfrm = cur_covisibilities.at(i);

        // camera center of the neighbor keyframe
        const Vec3_t ngh_cam_center = ngh_keyfrm->get_cam_center();

        // compute the baseline between the current and neighbor keyframes
        const Vec3_t baseline_vec = ngh_cam_center - cur_cam_center;
        const auto baseline_dist = baseline_vec.norm();
        if (is_monocular_) {
            // if the scene scale is much smaller than the baseline, abort the triangulation
            const float median_depth_in_ngh = ngh_keyfrm->compute_median_depth(true);
            if (baseline_dist < 0.02 * median_depth_in_ngh) {
                continue;
            }
        }
        else {
            // for stereo setups, it needs longer baseline than the stereo baseline
            if (baseline_dist < ngh_keyfrm->camera_->true_baseline_) {
                continue;
            }
        }

        // estimate matches between the current and neighbor keyframes,
        // then reject outliers using Essential matrix computed from the two camera poses

        // (cur bearing) * E_ngh_to_cur * (ngh bearing) = 0
        // const Mat33_t E_ngh_to_cur = solve::essential_solver::create_E_21(ngh_keyfrm, cur_keyfrm_);
        const Mat33_t E_ngh_to_cur = solve::essential_solver::create_E_21(ngh_keyfrm->get_rotation(), ngh_keyfrm->get_translation(),
                                                                          cur_keyfrm_->get_rotation(), cur_keyfrm_->get_translation());

        // vector of matches (idx in the current, idx in the neighbor)
        std::vector<std::pair<unsigned int, unsigned int>> matches;
        robust_matcher.match_for_triangulation(cur_keyfrm_, ngh_keyfrm, E_ngh_to_cur, matches);

        // triangulation
        triangulate_with_two_keyframes(cur_keyfrm_, ngh_keyfrm, matches);
    }
}

void mapping_module::triangulate_with_two_keyframes(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2,
                                                    const std::vector<std::pair<unsigned int, unsigned int>>& matches) {
    const module::two_view_triangulator triangulator(keyfrm_1, keyfrm_2, 1.0);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (unsigned int i = 0; i < matches.size(); ++i) {
        const auto idx_1 = matches.at(i).first;
        const auto idx_2 = matches.at(i).second;

        // triangulate between idx_1 and idx_2
        Vec3_t pos_w;
        if (!triangulator.triangulate(idx_1, idx_2, pos_w)) {
            // failed
            continue;
        }
        // succeeded

        // create a landmark object
        cv::Point2f pt = keyfrm_1->keypts_[idx_1].pt;
        cv::Vec<uchar, 3> color = keyfrm_1->frm_->img_->at<cv::Vec<uchar, 3>>(pt);
        auto lm = new data::landmark(pos_w, keyfrm_1, map_db_, color);

        lm->add_observation(keyfrm_1, idx_1);
        lm->add_observation(keyfrm_2, idx_2);

        keyfrm_1->add_landmark(lm, idx_1);
        keyfrm_2->add_landmark(lm, idx_2);

        lm->compute_descriptor();
        lm->update_normal_and_depth();

        map_db_->add_landmark(lm);
        // wait for redundancy check
#ifdef USE_OPENMP
#pragma omp critical
#endif
        {
            local_map_cleaner_->add_fresh_landmark(lm);
        }
    }
}

void mapping_module::update_new_keyframe() {
    // get the targets to check landmark fusion
    const auto fuse_tgt_keyfrms = get_second_order_covisibilities(is_monocular_ ? 20 : 10, 5);

    // resolve the duplication of landmarks between the current keyframe and the targets
    fuse_landmark_duplication(fuse_tgt_keyfrms);

    // update the geometries
    const auto cur_landmarks = cur_keyfrm_->get_landmarks();
    for (const auto lm : cur_landmarks) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        lm->compute_descriptor();
        lm->update_normal_and_depth();
    }

    // update the graph
    cur_keyfrm_->graph_node_->update_connections();
}

std::unordered_set<data::keyframe*> mapping_module::get_second_order_covisibilities(const unsigned int first_order_thr,
                                                                                    const unsigned int second_order_thr) {
    const auto cur_covisibilities = cur_keyfrm_->graph_node_->get_top_n_covisibilities(first_order_thr);

    std::unordered_set<data::keyframe*> fuse_tgt_keyfrms;
    fuse_tgt_keyfrms.reserve(cur_covisibilities.size() * 2);

    for (const auto first_order_covis : cur_covisibilities) {
        if (first_order_covis->will_be_erased()) {
            continue;
        }

        // check if the keyframe is aleady inserted
        if (static_cast<bool>(fuse_tgt_keyfrms.count(first_order_covis))) {
            continue;
        }

        fuse_tgt_keyfrms.insert(first_order_covis);

        // get the covisibilities of the covisibility of the current keyframe
        const auto ngh_covisibilities = first_order_covis->graph_node_->get_top_n_covisibilities(second_order_thr);
        for (const auto second_order_covis : ngh_covisibilities) {
            if (second_order_covis->will_be_erased()) {
                continue;
            }
            // "the covisibilities of the covisibility" contains the current keyframe
            if (*second_order_covis == *cur_keyfrm_) {
                continue;
            }

            fuse_tgt_keyfrms.insert(second_order_covis);
        }
    }

    return fuse_tgt_keyfrms;
}

void mapping_module::fuse_landmark_duplication(const std::unordered_set<data::keyframe*>& fuse_tgt_keyfrms) {
    match::fuse matcher;

    {
        // reproject the landmarks observed in the current keyframe to each of the targets, and acquire
        // - additional matches
        // - duplication of matches
        // then, add matches and solve duplication
        auto cur_landmarks = cur_keyfrm_->get_landmarks();
        for (const auto fuse_tgt_keyfrm : fuse_tgt_keyfrms) {
            matcher.replace_duplication(fuse_tgt_keyfrm, cur_landmarks);
        }
    }

    {
        // reproject the landmarks observed in each of the targets to each of the current frame, and acquire
        // - additional matches
        // - duplication of matches
        // then, add matches and solve duplication
        std::unordered_set<data::landmark*> candidate_landmarks_to_fuse;
        candidate_landmarks_to_fuse.reserve(fuse_tgt_keyfrms.size() * cur_keyfrm_->num_keypts_);

        for (const auto fuse_tgt_keyfrm : fuse_tgt_keyfrms) {
            const auto fuse_tgt_landmarks = fuse_tgt_keyfrm->get_landmarks();

            for (const auto lm : fuse_tgt_landmarks) {
                if (!lm) {
                    continue;
                }
                if (lm->will_be_erased()) {
                    continue;
                }

                if (static_cast<bool>(candidate_landmarks_to_fuse.count(lm))) {
                    continue;
                }
                candidate_landmarks_to_fuse.insert(lm);
            }
        }

        matcher.replace_duplication(cur_keyfrm_, candidate_landmarks_to_fuse);
    }
}

void mapping_module::request_reset() {
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

bool mapping_module::reset_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    return reset_is_requested_;
}

void mapping_module::reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    spdlog::info("reset mapping module");
    keyfrms_queue_.clear();
    local_map_cleaner_->reset();
    reset_is_requested_ = false;
}

void mapping_module::request_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
    std::lock_guard<std::mutex> lock2(mtx_keyfrm_queue_);
    abort_local_BA_ = true;
}

bool mapping_module::is_paused() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

bool mapping_module::pause_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_ && !force_to_run_;
}

void mapping_module::pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    spdlog::info("pause mapping module");
    is_paused_ = true;
}

bool mapping_module::set_force_to_run(const bool force_to_run) {
    std::lock_guard<std::mutex> lock(mtx_pause_);

    if (force_to_run && is_paused_) {
        return false;
    }

    force_to_run_ = force_to_run;
    return true;
}

void mapping_module::resume() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    std::lock_guard<std::mutex> lock2(mtx_terminate_);

    // if it has been already terminated, cannot resume
    if (is_terminated_) {
        return;
    }

    is_paused_ = false;
    pause_is_requested_ = false;

    // clear the queue
    for (auto& new_keyframe : keyfrms_queue_) {
        delete new_keyframe;
    }
    keyfrms_queue_.clear();

    spdlog::info("resume mapping module");
}

void mapping_module::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool mapping_module::is_terminated() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool mapping_module::terminate_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void mapping_module::terminate() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    std::lock_guard<std::mutex> lock2(mtx_terminate_);
    is_paused_ = true;
    is_terminated_ = true;
}

} // namespace openvslam
