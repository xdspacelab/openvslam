#include "openvslam/type.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/map/local_mapper.h"
#include "openvslam/map/two_view_triangulator.h"
#include "openvslam/map/loop_closer.h"
#include "openvslam/match/fuse.h"
#include "openvslam/match/robust.h"
#include "openvslam/solver/essential_solver.h"
#include "openvslam/solver/triangulator.h"

#include <unordered_set>
#include <thread>

#include <spdlog/spdlog.h>

namespace openvslam {
namespace map {

local_mapper::local_mapper(data::map_database* map_db, const bool is_monocular)
        : map_db_(map_db), local_bundle_adjuster_(), is_monocular_(is_monocular) {
    spdlog::debug("CONSTRUCT: map::local_mapper");
}

local_mapper::~local_mapper() {
    spdlog::debug("DESTRUCT: map::local_mapper");
}

void local_mapper::set_tracker(track::tracker* tracker) {
    tracker_ = tracker;
}

void local_mapper::set_loop_closer(loop_closer* loop_closer) {
    loop_closer_ = loop_closer;
}

void local_mapper::run() {
    spdlog::info("start mapping module");

    is_terminated_ = false;

    while (true) {
        // waiting time for the other threads
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // LOCK
        set_keyframe_acceptability(false);

        // check if termination is requested
        if (check_terminate_request()) {
            // terminate and break
            terminate();
            break;
        }

        // check if pause is requested
        if (check_pause_request()) {
            // if any keyframe is queued, all of them must be processed before the pause
            while (keyframe_is_queued()) {
                // create and extend the map with the new keyframe
                mapping_with_new_keyframe();
                // send the new keyframe to the loop closer
                loop_closer_->queue_keyframe(cur_keyfrm_);
            }
            // pause and wait
            pause();
            // check if termination or reset is requested during pause
            while (is_paused() && !check_terminate_request() && !check_reset_request()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        }

        // check if reset is requested
        if (check_reset_request()) {
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
        // send the new keyframe to the loop closer
        loop_closer_->queue_keyframe(cur_keyfrm_);

        // LOCK end
        set_keyframe_acceptability(true);
    }

    spdlog::info("terminate mapping module");
}

void local_mapper::queue_keyframe(data::keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    keyfrms_queue_.push_back(keyfrm);
    abort_BA_is_requested_ = true;
}

unsigned int local_mapper::get_num_queued_keyframes() const {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    return keyfrms_queue_.size();
}

bool local_mapper::keyframe_is_queued() const {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    return !keyfrms_queue_.empty();
}

bool local_mapper::get_keyframe_acceptability() const {
    return keyframe_acceptability_;
}

void local_mapper::set_keyframe_acceptability(const bool acceptability) {
    keyframe_acceptability_ = acceptability;
}

void local_mapper::abort_local_BA() {
    abort_BA_is_requested_ = true;
}

void local_mapper::mapping_with_new_keyframe() {
    // dequeue
    {
        std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
        // dequeue -> cur_keyfrm_
        cur_keyfrm_ = keyfrms_queue_.front();
        keyfrms_queue_.pop_front();
    }

    // store the new keyframe to the database
    store_new_keyframe();

    // remove redundant landmarks
    remove_redundant_landmarks();

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
    abort_BA_is_requested_ = false;
    if (2 < map_db_->get_num_keyframes()) {
        local_bundle_adjuster_.optimize(cur_keyfrm_, &abort_BA_is_requested_);
    }
    remove_redundant_keyframes();
}

void local_mapper::store_new_keyframe() {
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
            continue;
        }

        // update connection
        lm->add_observation(cur_keyfrm_, idx);
        // update geometry
        lm->update_normal_and_depth();
        lm->compute_descriptor();
    }
    cur_keyfrm_->update_connections();

    // store the new keyframe to the map database
    map_db_->add_keyframe(cur_keyfrm_);
}

void local_mapper::create_new_landmarks() {
    // get the covisibilities of `cur_keyfrm_`
    // in order to triangulate landmarks between `cur_keyfrm_` and each of the covisibilities
    constexpr unsigned int num_covisibilities = 10;
    const auto cur_covisibilities = cur_keyfrm_->get_top_n_covisibilities(num_covisibilities * (is_monocular_ ?  2 : 1));

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
        const Mat33_t E_ngh_to_cur = solver::essential_solver::create_E_21(ngh_keyfrm, cur_keyfrm_);

        // vector of matches (idx in the current, idx in the neighbor)
        std::vector<std::pair<unsigned int, unsigned int>> matches;
        robust_matcher.match_for_triangulation(cur_keyfrm_, ngh_keyfrm, E_ngh_to_cur, matches);

        // triangulation
        triangulate_with_two_keyframes(cur_keyfrm_, ngh_keyfrm, matches);
    }
}

void local_mapper::triangulate_with_two_keyframes(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2,
                                                  const std::vector<std::pair<unsigned int, unsigned int>>& matches) {
    const two_view_triangulator triangulator(keyfrm_1, keyfrm_2, 1.0);

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (unsigned int j = 0; j < matches.size(); ++j) {
        const auto idx_1 = matches.at(j).first;
        const auto idx_2 = matches.at(j).second;

        // triangulate between idx_1 and idx_2
        Vec3_t pos_w;
        if (!triangulator.triangulate(idx_1, idx_2, pos_w)) {
            // failed
            continue;
        }
        // succeeded

        // create a landmark object
        auto lm = new data::landmark(pos_w, keyfrm_1, map_db_);

        lm->add_observation(keyfrm_1, idx_1);
        lm->add_observation(keyfrm_2, idx_2);

        keyfrm_1->add_landmark(lm, idx_1);
        keyfrm_2->add_landmark(lm, idx_2);

        lm->compute_descriptor();
        lm->update_normal_and_depth();

        map_db_->add_landmark(lm);
        // wait for redundancy check
#ifdef _OPENMP
#pragma omp critical
#endif
        {
            fresh_landmarks_.push_back(lm);
        }
    }
}

void local_mapper::remove_redundant_landmarks() {
    constexpr float found_per_visible_ratio_thr = 0.25;
    constexpr unsigned int num_reliable_keyfrms = 2;
    const unsigned int num_obs_thr = is_monocular_ ? 2 : 3;

    auto iter = fresh_landmarks_.begin();
    const unsigned int cur_keyfrm_id = cur_keyfrm_->id_;

    // states of observed landmarks
    enum class lm_state_t {Valid, Invalid, NotClear};

    while (iter != fresh_landmarks_.end()) {
        auto lm = *iter;

        // decide the state of lms the buffer
        auto lm_state = lm_state_t::NotClear;
        if (lm->will_be_erased()) {
            // in case `lm` will be erased
            // remove `lm` from the buffer
            lm_state = lm_state_t::Valid;
        }
        else if (lm->get_found_per_visible_ratio() < found_per_visible_ratio_thr) {
            // if `lm` is not reliable
            // remove `lm` from the buffer and the database
            lm_state = lm_state_t::Invalid;
        }
        else if (num_reliable_keyfrms + lm->first_keyfrm_id_ <= cur_keyfrm_id
                 && lm->num_observations() <= num_obs_thr) {
            // if the number of the observers of `lm` is small after some keyframes were inserted
            // remove `lm` from the buffer and the database
            lm_state = lm_state_t::Invalid;
        }
        else if (num_reliable_keyfrms + 1U + lm->first_keyfrm_id_ <= cur_keyfrm_id) {
            // if the number of the observers of `lm` is sufficient after some keyframes were inserted
            // remove `lm` from the buffer
            lm_state = lm_state_t::Valid;
        }

        // select to remove `lm` according to the state
        if (lm_state == lm_state_t::Valid) {
            iter = fresh_landmarks_.erase(iter);
        }
        else if (lm_state == lm_state_t::Invalid) {
            lm->prepare_for_erasing();
            iter = fresh_landmarks_.erase(iter);
        }
        else {
            // hold decision because the state is NotClear
            iter++;
        }
    }
}

void local_mapper::remove_redundant_keyframes() {
    // window size not to remove
    constexpr unsigned int window_size_not_to_remove = 2;
    // if the redundancy ratio of observations is larger than this threshold,
    // the corresponding keyframe will be erased
    constexpr float redundant_obs_ratio_thr = 0.9;

    // check redundancy for each of the covisibilities
    const auto cur_covisibilities = cur_keyfrm_->get_covisibilities();
    for (const auto covisibility : cur_covisibilities) {
        // cannot remove the origin
        if (*covisibility == *(map_db_->origin_keyfrm_)) {
            continue;
        }
        // cannot remove the recent keyframe(s)
        if (covisibility->id_ <= cur_keyfrm_->id_
            && cur_keyfrm_->id_ <= covisibility->id_ + window_size_not_to_remove) {
            continue;
        }

        // count the number of redundant observations (num_redundant_obs) and valid observations (num_valid_obs)
        // for the covisibility
        unsigned int num_redundant_obs = 0;
        unsigned int num_valid_obs = 0;
        count_redundant_landmarks(covisibility, num_valid_obs, num_redundant_obs);

        // if the redundant observation ratio of `covisibility` is larger than the threshold, it will be removed
        if (redundant_obs_ratio_thr <= static_cast<float>(num_redundant_obs) / num_valid_obs) {
            covisibility->prepare_for_erasing();
        }
    }
}

void local_mapper::count_redundant_landmarks(data::keyframe* keyfrm, unsigned int& num_valid_obs, unsigned int& num_redundant_obs) const {
    // if the number of keyframes that observes the landmark with more reliable scale than the specified keyframe does,
    // it is considered as redundant
    constexpr unsigned int num_better_obs_thr = 3;

    num_valid_obs = 0;
    num_redundant_obs = 0;

    const auto landmarks = keyfrm->get_landmarks();
    for (unsigned int idx = 0; idx < landmarks.size(); ++idx) {
        auto lm = landmarks.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // if depth is within the valid range, it won't be considered
        const auto depth = keyfrm->depths_.at(idx);
        if (!is_monocular_ && (depth < 0.0 || keyfrm->depth_thr_ < depth)) {
            continue;
        }

        ++num_valid_obs;

        // if the number of the obs is smaller than the threshold, cannot remote the observers
        if (lm->num_observations() <= num_better_obs_thr) {
            continue;
        }

        // `keyfrm` observes `lm` with the scale level `scale_level`
        const auto scale_level = keyfrm->undist_keypts_.at(idx).octave;
        // get observers of `lm`
        const auto observations = lm->get_observations();

        bool obs_by_keyfrm_is_redundant = false;

        // the number of the keyframes that observe `lm` with the more reliable (closer) scale
        unsigned int num_better_obs = 0;

        for (const auto obs : observations) {
            const auto ngh_keyfrm = obs.first;
            if (*ngh_keyfrm == *keyfrm) {
                continue;
            }

            // `ngh_keyfrm` observes `lm` with the scale level `ngh_scale_level`
            const auto ngh_scale_level = ngh_keyfrm->undist_keypts_.at(obs.second).octave;

            // compare the scale levels
            if (ngh_scale_level <= scale_level + 1) {
                // the observation by `ngh_keyfrm` is more reliable than `keyfrm`
                ++num_better_obs;
                if (num_better_obs_thr <= num_better_obs) {
                    // if the number of the better observations is greater than the threshold,
                    // consider the observation of `lm` by `keyfrm` is redundant
                    obs_by_keyfrm_is_redundant = true;
                    break;
                }
            }
        }

        if (obs_by_keyfrm_is_redundant) {
            // keyfrmが観測している3次元点のうち，冗長と判断されたものの数を数えておく
            ++num_redundant_obs;
        }
    }
}

void local_mapper::update_new_keyframe() {
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
    cur_keyfrm_->update_connections();
}

std::unordered_set<data::keyframe*> local_mapper::get_second_order_covisibilities(const unsigned int first_order_thr,
                                                                                  const unsigned int second_order_thr) {
    const auto cur_covisibilities = cur_keyfrm_->get_top_n_covisibilities(first_order_thr);

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
        const auto ngh_covisibilities = first_order_covis->get_top_n_covisibilities(second_order_thr);
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

void local_mapper::fuse_landmark_duplication(const std::unordered_set<data::keyframe*>& fuse_tgt_keyfrms) {
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

void local_mapper::request_reset() {
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

bool local_mapper::check_reset_request() const {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    return reset_is_requested_;
}

void local_mapper::reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    spdlog::info("reset mapping module");
    keyfrms_queue_.clear();
    fresh_landmarks_.clear();
    reset_is_requested_ = false;
}

void local_mapper::request_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
    std::lock_guard<std::mutex> lock2(mtx_keyfrm_queue_);
    abort_BA_is_requested_ = true;
}

bool local_mapper::pause_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_;
}

bool local_mapper::is_paused() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

bool local_mapper::check_pause_request() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_ && !force_to_run_;
}

void local_mapper::pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    spdlog::info("pause mapping module");
    is_paused_ = true;
}

bool local_mapper::set_force_to_run(const bool force_to_run) {
    std::lock_guard<std::mutex> lock(mtx_pause_);

    if (force_to_run && is_paused_) {
        return false;
    }

    force_to_run_ = force_to_run;
    return true;
}

void local_mapper::resume() {
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

void local_mapper::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool local_mapper::is_terminated() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool local_mapper::check_terminate_request() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void local_mapper::terminate() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    std::lock_guard<std::mutex> lock2(mtx_terminate_);
    is_paused_ = true;
    is_terminated_ = true;
}

} // namespace map
} // namespace openvslam
