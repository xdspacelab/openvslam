#include "openvslam/mapping_module.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/data/map_database.h"
#include "openvslam/module/keyframe_inserter.h"

namespace openvslam {
namespace module {

keyframe_inserter::keyframe_inserter(const camera::setup_type_t setup_type, const float true_depth_thr,
                                     data::map_database* map_db, data::bow_database* bow_db,
                                     const unsigned int min_num_frms, const unsigned int max_num_frms)
    : setup_type_(setup_type), true_depth_thr_(true_depth_thr),
      map_db_(map_db), bow_db_(bow_db),
      min_num_frms_(min_num_frms), max_num_frms_(max_num_frms) {}

void keyframe_inserter::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
}

void keyframe_inserter::reset() {
    frm_id_of_last_keyfrm_ = 0;
}

bool keyframe_inserter::new_keyframe_is_needed(const data::frame& curr_frm, const unsigned int num_tracked_lms,
                                               const data::keyframe& ref_keyfrm) const {
    assert(mapper_);
    // Any keyframes are not able to be added when the mapping module stops
    if (mapper_->is_paused() || mapper_->pause_is_requested()) {
        return false;
    }

    const auto num_keyfrms = map_db_->get_num_keyframes();

    // Count the number of the 3D points that are observed from more than two keyframes
    const unsigned int min_obs_thr = (3 <= num_keyfrms) ? 3 : 2;
    const auto num_reliable_lms = ref_keyfrm.get_num_tracked_landmarks(min_obs_thr);

    // Check if the mapping is in progress or not
    const bool mapper_is_idle = mapper_->get_keyframe_acceptability();

    // Ratio-threshold of "the number of 3D points observed in the current frame" / "that of 3D points observed in the last keyframe"
    constexpr unsigned int num_tracked_lms_thr = 15;
    const float lms_ratio_thr = 0.9;

    // Condition A1: Add a keyframe if the number of frames added after the previous keyframe insertion reaches the threshold
    const bool cond_a1 = frm_id_of_last_keyfrm_ + max_num_frms_ <= curr_frm.id_;
    // Condition A2: Add a keyframe if the number of added frames exceeds the minimum,
    //               and concurrently the mapping module remains standing-by
    const bool cond_a2 = (frm_id_of_last_keyfrm_ + min_num_frms_ <= curr_frm.id_) && mapper_is_idle;
    // Condition A3: Add a keyframe if the field-of-view of the current frame is changed a lot
    const bool cond_a3 = num_tracked_lms < num_reliable_lms * 0.25;

    // Condition B: (Mandatory for keyframe insertion)
    //              Add a keyframe if the number of 3D points exceeds the threshold,
    //              and concurrently the ratio of the reliable 3D points larger than the threshold ratio
    const bool cond_b = (num_tracked_lms_thr <= num_tracked_lms) && (num_tracked_lms < num_reliable_lms * lms_ratio_thr);

    // Do not add any kerframes if the condition B is not satisfied
    if (!cond_b) {
        return false;
    }

    // Do not add any kerframes if all the conditions A are not satisfied
    if (!cond_a1 && !cond_a2 && !cond_a3) {
        return false;
    }

    // Add the keyframe if the mapping module isn't in the process
    if (mapper_is_idle) {
        return true;
    }

    // Stop the local bundle adjustment if the mapping module is in the process, then add a new keyframe
    if (setup_type_ != camera::setup_type_t::Monocular
        && mapper_->get_num_queued_keyframes() <= 2) {
        mapper_->abort_local_BA();
        return true;
    }

    return false;
}

data::keyframe* keyframe_inserter::insert_new_keyframe(data::frame& curr_frm) {
    // Force the mapping module to run
    if (!mapper_->set_force_to_run(true)) {
        return nullptr;
    }

    curr_frm.update_pose_params();
    auto keyfrm = new data::keyframe(curr_frm, map_db_, bow_db_);

    frm_id_of_last_keyfrm_ = curr_frm.id_;

    // Queue up the keyframe to the mapping module
    if (setup_type_ == camera::setup_type_t::Monocular) {
        queue_keyframe(keyfrm);
        return keyfrm;
    }

    // Save the valid depth and index pairs
    std::vector<std::pair<float, unsigned int>> depth_idx_pairs;
    depth_idx_pairs.reserve(curr_frm.num_keypts_);
    for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
        const auto depth = curr_frm.depths_.at(idx);
        // Add if the depth is valid
        if (0 < depth) {
            depth_idx_pairs.emplace_back(std::make_pair(depth, idx));
        }
    }

    // Queue up the keyframe to the mapping module if any valid depth values don't exist
    if (depth_idx_pairs.empty()) {
        queue_keyframe(keyfrm);
        return keyfrm;
    }

    // Sort in order of distance to the camera
    std::sort(depth_idx_pairs.begin(), depth_idx_pairs.end());

    // Create 3D points by using a depth parameter
    constexpr unsigned int min_num_to_create = 100;
    for (unsigned int count = 0; count < depth_idx_pairs.size(); ++count) {
        const auto depth = depth_idx_pairs.at(count).first;
        const auto idx = depth_idx_pairs.at(count).second;

        // Stop adding a keyframe if the number of 3D points exceeds the minimal threshold,
        // and concurrently the depth value exceeds the threshold
        if (min_num_to_create < count && true_depth_thr_ < depth) {
            break;
        }

        // Stereo-triangulation cannot be performed if the 3D point has been already associated to the keypoint index
        {
            auto lm = curr_frm.landmarks_.at(idx);
            if (lm) {
                assert(lm->has_observation());
                continue;
            }
        }

        // Stereo-triangulation can be performed if the 3D point is not yet associated to the keypoint index
        const Vec3_t pos_w = curr_frm.triangulate_stereo(idx);
        auto lm = new data::landmark(pos_w, keyfrm, map_db_);

        lm->add_observation(keyfrm, idx);
        keyfrm->add_landmark(lm, idx);
        curr_frm.landmarks_.at(idx) = lm;

        lm->compute_descriptor();
        lm->update_normal_and_depth();

        map_db_->add_landmark(lm);
    }

    // Queue up the keyframe to the mapping module
    queue_keyframe(keyfrm);
    return keyfrm;
}

void keyframe_inserter::queue_keyframe(data::keyframe* keyfrm) {
    mapper_->queue_keyframe(keyfrm);
    mapper_->set_force_to_run(false);
}

} // namespace module
} // namespace openvslam
