#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/module/local_map_updater.h"

namespace openvslam {
namespace module {

local_map_updater::local_map_updater(const data::frame& curr_frm, const unsigned int max_num_local_keyfrms)
    : frm_id_(curr_frm.id_), frm_lms_(curr_frm.landmarks_), num_keypts_(curr_frm.num_keypts_),
      max_num_local_keyfrms_(max_num_local_keyfrms) {}

std::vector<data::keyframe*> local_map_updater::get_local_keyframes() const {
    return local_keyfrms_;
}

std::vector<data::landmark*> local_map_updater::get_local_landmarks() const {
    return local_lms_;
}

data::keyframe* local_map_updater::get_nearest_covisibility() const {
    return nearest_covisibility_;
}

bool local_map_updater::acquire_local_map() {
    const auto local_keyfrms_was_found = find_local_keyframes();
    const auto local_lms_was_found = find_local_landmarks();
    return local_keyfrms_was_found && local_lms_was_found;
}

bool local_map_updater::find_local_keyframes() {
    const auto keyfrm_weights = count_keyframe_weights();
    if (keyfrm_weights.empty()) {
        return false;
    }
    const auto first_local_keyfrms = find_first_local_keyframes(keyfrm_weights);
    const auto second_local_keyfrms = find_second_local_keyframes(first_local_keyfrms);
    local_keyfrms_ = first_local_keyfrms;
    std::copy(second_local_keyfrms.begin(), second_local_keyfrms.end(), std::back_inserter(local_keyfrms_));
    return true;
}

local_map_updater::keyframe_weights_t local_map_updater::count_keyframe_weights() const {
    // count the number of sharing landmarks between the current frame and each of the neighbor keyframes
    // key: keyframe, value: number of sharing landmarks
    keyframe_weights_t keyfrm_weights;
    for (unsigned int idx = 0; idx < num_keypts_; ++idx) {
        auto lm = frm_lms_.at(idx);
        if (!lm) {
            continue;
        }
        const auto observations = lm->get_observations();
        for (auto obs : observations) {
            ++keyfrm_weights[obs.first];
        }
    }
    return keyfrm_weights;
}

auto local_map_updater::find_first_local_keyframes(const keyframe_weights_t& keyfrm_weights)
    -> std::vector<data::keyframe*> {
    std::vector<data::keyframe*> first_local_keyfrms;
    first_local_keyfrms.reserve(2 * keyfrm_weights.size());

    unsigned int max_weight = 0;
    for (auto& keyfrm_weight : keyfrm_weights) {
        auto keyfrm = keyfrm_weight.first;
        const auto weight = keyfrm_weight.second;

        if (keyfrm->will_be_erased()) {
            continue;
        }

        first_local_keyfrms.push_back(keyfrm);

        // avoid duplication
        keyfrm->local_map_update_identifier = frm_id_;

        // update the nearest keyframe
        if (max_weight < weight) {
            max_weight = weight;
            nearest_covisibility_ = keyfrm;
        }
    }

    return first_local_keyfrms;
}

auto local_map_updater::find_second_local_keyframes(const std::vector<data::keyframe*>& first_local_keyframes) const
    -> std::vector<data::keyframe*> {
    std::vector<data::keyframe*> second_local_keyfrms;
    second_local_keyfrms.reserve(4 * first_local_keyframes.size());

    // add the second-order keyframes to the local landmarks
    auto add_second_local_keyframe = [this, &second_local_keyfrms](data::keyframe* keyfrm) {
        if (!keyfrm) {
            return false;
        }
        if (keyfrm->will_be_erased()) {
            return false;
        }
        // avoid duplication
        if (keyfrm->local_map_update_identifier == frm_id_) {
            return false;
        }
        keyfrm->local_map_update_identifier = frm_id_;
        second_local_keyfrms.push_back(keyfrm);
        return true;
    };
    for (auto iter = first_local_keyframes.cbegin(); iter != first_local_keyframes.cend(); ++iter) {
        if (max_num_local_keyfrms_ < first_local_keyframes.size() + second_local_keyfrms.size()) {
            break;
        }

        auto keyfrm = *iter;

        // covisibilities of the neighbor keyframe
        const auto neighbors = keyfrm->graph_node_->get_top_n_covisibilities(10);
        for (auto neighbor : neighbors) {
            if (add_second_local_keyframe(neighbor)) {
                break;
            }
        }

        // children of the spanning tree
        const auto spanning_children = keyfrm->graph_node_->get_spanning_children();
        for (auto child : spanning_children) {
            if (add_second_local_keyframe(child)) {
                break;
            }
        }

        // parent of the spanning tree
        auto parent = keyfrm->graph_node_->get_spanning_parent();
        add_second_local_keyframe(parent);
    }

    return second_local_keyfrms;
}

bool local_map_updater::find_local_landmarks() {
    local_lms_.clear();
    local_lms_.reserve(50 * local_keyfrms_.size());

    for (auto keyfrm : local_keyfrms_) {
        const auto lms = keyfrm->get_landmarks();

        for (auto lm : lms) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            // avoid duplication
            if (lm->identifier_in_local_map_update_ == frm_id_) {
                continue;
            }
            lm->identifier_in_local_map_update_ = frm_id_;

            local_lms_.push_back(lm);
        }
    }

    return true;
}

} // namespace module
} // namespace openvslam
