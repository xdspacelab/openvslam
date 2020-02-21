#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/match/base.h"

#include <nlohmann/json.hpp>

namespace openvslam {
namespace data {

std::atomic<unsigned int> landmark::next_id_{0};

landmark::landmark(const Vec3_t& pos_w, keyframe* ref_keyfrm, map_database* map_db)
    : id_(next_id_++), first_keyfrm_id_(ref_keyfrm->id_), pos_w_(pos_w),
      ref_keyfrm_(ref_keyfrm), map_db_(map_db) {}

landmark::landmark(const unsigned int id, const unsigned int first_keyfrm_id,
                   const Vec3_t& pos_w, keyframe* ref_keyfrm,
                   const unsigned int num_visible, const unsigned int num_found,
                   map_database* map_db)
    : id_(id), first_keyfrm_id_(first_keyfrm_id), pos_w_(pos_w), ref_keyfrm_(ref_keyfrm),
      num_observable_(num_visible), num_observed_(num_found), map_db_(map_db) {}

void landmark::set_pos_in_world(const Vec3_t& pos_w) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    pos_w_ = pos_w;
}

Vec3_t landmark::get_pos_in_world() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return pos_w_;
}

Vec3_t landmark::get_obs_mean_normal() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return mean_normal_;
}

keyframe* landmark::get_ref_keyframe() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return ref_keyfrm_;
}

void landmark::add_observation(keyframe* keyfrm, unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    if (observations_.count(keyfrm)) {
        return;
    }
    observations_[keyfrm] = idx;

    if (0 <= keyfrm->stereo_x_right_.at(idx)) {
        num_observations_ += 2;
    }
    else {
        num_observations_ += 1;
    }
}

void landmark::erase_observation(keyframe* keyfrm) {
    bool discard = false;
    {
        std::lock_guard<std::mutex> lock(mtx_observations_);

        if (observations_.count(keyfrm)) {
            int idx = observations_.at(keyfrm);
            if (0 <= keyfrm->stereo_x_right_.at(idx)) {
                num_observations_ -= 2;
            }
            else {
                num_observations_ -= 1;
            }

            observations_.erase(keyfrm);

            if (ref_keyfrm_ == keyfrm) {
                ref_keyfrm_ = observations_.begin()->first;
            }

            // If only 2 observations or less, discard point
            if (num_observations_ <= 2) {
                discard = true;
            }
        }
    }

    if (discard) {
        prepare_for_erasing();
    }
}

std::map<keyframe*, unsigned int> landmark::get_observations() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return observations_;
}

unsigned int landmark::num_observations() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observations_;
}

bool landmark::has_observation() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return 0 < num_observations_;
}

int landmark::get_index_in_keyframe(keyframe* keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    if (observations_.count(keyfrm)) {
        return observations_.at(keyfrm);
    }
    else {
        return -1;
    }
}

bool landmark::is_observed_in_keyframe(keyframe* keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return static_cast<bool>(observations_.count(keyfrm));
}

cv::Mat landmark::get_descriptor() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return descriptor_.clone();
}

void landmark::compute_descriptor() {
    std::map<keyframe*, unsigned int> observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        if (will_be_erased_) {
            return;
        }
        observations = observations_;
    }

    if (observations.empty()) {
        return;
    }

    // Append features of corresponding points
    std::vector<cv::Mat> descriptors;
    descriptors.reserve(observations.size());
    for (const auto& observation : observations) {
        auto keyfrm = observation.first;
        const auto idx = observation.second;

        if (!keyfrm->will_be_erased()) {
            descriptors.push_back(keyfrm->descriptors_.row(idx));
        }
    }

    // Get median of Hamming distance
    // Calculate all the Hamming distances between every pair of the features
    const auto num_descs = descriptors.size();
    std::vector<std::vector<unsigned int>> hamm_dists(num_descs, std::vector<unsigned int>(num_descs));
    for (unsigned int i = 0; i < num_descs; ++i) {
        hamm_dists.at(i).at(i) = 0;
        for (unsigned int j = i + 1; j < num_descs; ++j) {
            const auto dist = match::compute_descriptor_distance_32(descriptors.at(i), descriptors.at(j));
            hamm_dists.at(i).at(j) = dist;
            hamm_dists.at(j).at(i) = dist;
        }
    }

    // Get the nearest value to median
    unsigned int best_median_dist = match::MAX_HAMMING_DIST;
    unsigned int best_idx = 0;
    for (unsigned idx = 0; idx < num_descs; ++idx) {
        std::vector<unsigned int> partial_hamm_dists(hamm_dists.at(idx).begin(), hamm_dists.at(idx).begin() + num_descs);
        std::sort(partial_hamm_dists.begin(), partial_hamm_dists.end());
        const auto median_dist = partial_hamm_dists.at(static_cast<unsigned int>(0.5 * (num_descs - 1)));

        if (median_dist < best_median_dist) {
            best_median_dist = median_dist;
            best_idx = idx;
        }
    }

    {
        std::lock_guard<std::mutex> lock(mtx_observations_);
        descriptor_ = descriptors.at(best_idx).clone();
    }
}

void landmark::update_normal_and_depth() {
    std::map<keyframe*, unsigned int> observations;
    keyframe* ref_keyfrm;
    Vec3_t pos_w;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        std::lock_guard<std::mutex> lock2(mtx_position_);
        if (will_be_erased_) {
            return;
        }
        observations = observations_;
        ref_keyfrm = ref_keyfrm_;
        pos_w = pos_w_;
    }

    if (observations.empty()) {
        return;
    }

    Vec3_t mean_normal = Vec3_t::Zero();
    unsigned int num_observations = 0;
    for (const auto& observation : observations) {
        auto keyfrm = observation.first;
        const Vec3_t cam_center = keyfrm->get_cam_center();
        const Vec3_t normal = pos_w_ - cam_center;
        mean_normal = mean_normal + normal.normalized();
        ++num_observations;
    }

    const Vec3_t cam_to_lm_vec = pos_w - ref_keyfrm->get_cam_center();
    const auto dist = cam_to_lm_vec.norm();
    const auto scale_level = ref_keyfrm->undist_keypts_.at(observations.at(ref_keyfrm)).octave;
    const auto scale_factor = ref_keyfrm->scale_factors_.at(scale_level);
    const auto num_scale_levels = ref_keyfrm->num_scale_levels_;

    {
        std::lock_guard<std::mutex> lock3(mtx_position_);
        max_valid_dist_ = dist * scale_factor;
        min_valid_dist_ = max_valid_dist_ / ref_keyfrm->scale_factors_.at(num_scale_levels - 1);
        mean_normal_ = mean_normal.normalized();
    }
}

float landmark::get_min_valid_distance() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return 0.7 * min_valid_dist_;
}

float landmark::get_max_valid_distance() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return 1.3 * max_valid_dist_;
}

unsigned int landmark::predict_scale_level(const float cam_to_lm_dist, const frame* frm) const {
    float ratio;
    {
        std::lock_guard<std::mutex> lock(mtx_position_);
        ratio = max_valid_dist_ / cam_to_lm_dist;
    }

    const auto pred_scale_level = static_cast<int>(std::ceil(std::log(ratio) / frm->log_scale_factor_));
    if (pred_scale_level < 0) {
        return 0;
    }
    else if (frm->num_scale_levels_ <= static_cast<unsigned int>(pred_scale_level)) {
        return frm->num_scale_levels_ - 1;
    }
    else {
        return static_cast<unsigned int>(pred_scale_level);
    }
}

unsigned int landmark::predict_scale_level(const float cam_to_lm_dist, const keyframe* keyfrm) const {
    float ratio;
    {
        std::lock_guard<std::mutex> lock(mtx_position_);
        ratio = max_valid_dist_ / cam_to_lm_dist;
    }

    const auto pred_scale_level = static_cast<int>(std::ceil(std::log(ratio) / keyfrm->log_scale_factor_));
    if (pred_scale_level < 0) {
        return 0;
    }
    else if (keyfrm->num_scale_levels_ <= static_cast<unsigned int>(pred_scale_level)) {
        return keyfrm->num_scale_levels_ - 1;
    }
    else {
        return static_cast<unsigned int>(pred_scale_level);
    }
}

void landmark::prepare_for_erasing() {
    std::map<keyframe*, unsigned int> observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        std::lock_guard<std::mutex> lock2(mtx_position_);
        observations = observations_;
        observations_.clear();
        will_be_erased_ = true;
    }

    for (const auto& keyfrm_and_idx : observations) {
        keyfrm_and_idx.first->erase_landmark_with_index(keyfrm_and_idx.second);
    }

    map_db_->erase_landmark(this);
}

bool landmark::will_be_erased() {
    std::lock_guard<std::mutex> lock1(mtx_observations_);
    std::lock_guard<std::mutex> lock2(mtx_position_);
    return will_be_erased_;
}

void landmark::replace(landmark* lm) {
    if (lm->id_ == this->id_) {
        return;
    }

    unsigned int num_observable, num_observed;
    std::map<keyframe*, unsigned int> observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        std::lock_guard<std::mutex> lock2(mtx_position_);
        observations = observations_;
        observations_.clear();
        will_be_erased_ = true;
        num_observable = num_observable_;
        num_observed = num_observed_;
        replaced_ = lm;
    }

    for (const auto& keyfrm_and_idx : observations) {
        keyframe* keyfrm = keyfrm_and_idx.first;

        if (!lm->is_observed_in_keyframe(keyfrm)) {
            keyfrm->replace_landmark(lm, keyfrm_and_idx.second);
            lm->add_observation(keyfrm, keyfrm_and_idx.second);
        }
        else {
            keyfrm->erase_landmark_with_index(keyfrm_and_idx.second);
        }
    }

    lm->increase_num_observed(num_observed);
    lm->increase_num_observable(num_observable);
    lm->compute_descriptor();

    map_db_->erase_landmark(this);
}

landmark* landmark::get_replaced() const {
    std::lock_guard<std::mutex> lock1(mtx_observations_);
    std::lock_guard<std::mutex> lock2(mtx_position_);
    return replaced_;
}

void landmark::increase_num_observable(unsigned int num_observable) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    num_observable_ += num_observable;
}

void landmark::increase_num_observed(unsigned int num_observed) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    num_observed_ += num_observed;
}

float landmark::get_observed_ratio() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return static_cast<float>(num_observed_) / num_observable_;
}

nlohmann::json landmark::to_json() const {
    return {{"1st_keyfrm", first_keyfrm_id_},
            {"pos_w", {pos_w_(0), pos_w_(1), pos_w_(2)}},
            {"ref_keyfrm", ref_keyfrm_->id_},
            {"n_vis", num_observable_},
            {"n_fnd", num_observed_}};
}

} // namespace data
} // namespace openvslam
