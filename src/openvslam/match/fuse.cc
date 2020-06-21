#include "openvslam/match/fuse.h"
#include "openvslam/camera/base.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"

#include <vector>
#include <unordered_set>

namespace openvslam {
namespace match {

unsigned int fuse::detect_duplication(data::keyframe* keyfrm, const Mat44_t& Sim3_cw, const std::vector<data::landmark*>& landmarks_to_check,
                                      const float margin, std::vector<data::landmark*>& duplicated_lms_in_keyfrm) {
    unsigned int num_fused = 0;

    // Convert Sim3 into SE3
    const Mat33_t s_rot_cw = Sim3_cw.block<3, 3>(0, 0);
    const auto s_cw = std::sqrt(s_rot_cw.block<1, 3>(0, 0).dot(s_rot_cw.block<1, 3>(0, 0)));
    const Mat33_t rot_cw = s_rot_cw / s_cw;
    const Vec3_t trans_cw = Sim3_cw.block<3, 1>(0, 3) / s_cw;
    const Vec3_t cam_center = -rot_cw.transpose() * trans_cw;

    duplicated_lms_in_keyfrm = std::vector<data::landmark*>(landmarks_to_check.size(), nullptr);

    const auto valid_lms_in_keyfrm = keyfrm->get_valid_landmarks();

    for (unsigned int i = 0; i < landmarks_to_check.size(); ++i) {
        auto lm = landmarks_to_check.at(i);
        if (lm->will_be_erased()) {
            continue;
        }
        // Ignore if the 3D point maches to any keypoint because neither reprojection nor fusion is needed
        if (valid_lms_in_keyfrm.count(lm)) {
            continue;
        }

        // 3D point coordinates with the global reference
        const Vec3_t pos_w = lm->get_pos_in_world();

        // Reproject and compute visibility
        Vec2_t reproj;
        float x_right;
        const bool in_image = keyfrm->camera_->reproject_to_image(rot_cw, trans_cw, pos_w, reproj, x_right);

        // Ignore if it is reprojected outside the image
        if (!in_image) {
            continue;
        }

        // Check if it's within ORB scale levels
        const Vec3_t cam_to_lm_vec = pos_w - cam_center;
        const auto cam_to_lm_dist = cam_to_lm_vec.norm();
        const auto max_cam_to_lm_dist = lm->get_max_valid_distance();
        const auto min_cam_to_lm_dist = lm->get_min_valid_distance();

        if (cam_to_lm_dist < min_cam_to_lm_dist || max_cam_to_lm_dist < cam_to_lm_dist) {
            continue;
        }

        // Compute the angle formed by the average vector of the 3D point observation,
        // and discard it if it is wider than the threshold value (60 degrees)
        const Vec3_t obs_mean_normal = lm->get_obs_mean_normal();

        if (cam_to_lm_vec.dot(obs_mean_normal) < 0.5 * cam_to_lm_dist) {
            continue;
        }

        // Acquire keypoints in the cell where the reprojected 3D points exist
        const int pred_scale_level = lm->predict_scale_level(cam_to_lm_dist, keyfrm);
        const auto indices = keyfrm->get_keypoints_in_cell(reproj(0), reproj(1), margin * keyfrm->scale_factors_.at(pred_scale_level));

        if (indices.empty()) {
            continue;
        }

        // Find keypoints with the closest descriptor
        const auto lm_desc = lm->get_descriptor();

        unsigned int best_dist = MAX_HAMMING_DIST;
        int best_idx = -1;

        for (const auto idx : indices) {
            const auto scale_level = keyfrm->keypts_.at(idx).octave;

            // TODO: shoud determine the scale with 'keyfrm-> get_keypts_in_cell ()'
            if (scale_level < pred_scale_level - 1 || pred_scale_level < scale_level) {
                continue;
            }

            const auto& desc = keyfrm->descriptors_.row(idx);

            const auto hamm_dist = compute_descriptor_distance_32(lm_desc, desc);

            if (hamm_dist < best_dist) {
                best_dist = hamm_dist;
                best_idx = idx;
            }
        }

        if (HAMMING_DIST_THR_LOW < best_dist) {
            continue;
        }

        auto lm_in_keyfrm = keyfrm->get_landmark(best_idx);
        if (lm_in_keyfrm) {
            // There is association between the 3D point and the keyframe
            // -> Duplication exists
            if (!lm_in_keyfrm->will_be_erased()) {
                duplicated_lms_in_keyfrm.at(i) = lm_in_keyfrm;
            }
        }
        else {
            // There is no association between the 3D point and the keyframe
            // Add the observation information
            lm->add_observation(keyfrm, best_idx);
            keyfrm->add_landmark(lm, best_idx);
        }

        ++num_fused;
    }

    return num_fused;
}

template<typename T>
unsigned int fuse::replace_duplication(data::keyframe* keyfrm, const T& landmarks_to_check, const float margin) {
    unsigned int num_fused = 0;

    const Mat33_t rot_cw = keyfrm->get_rotation();
    const Vec3_t trans_cw = keyfrm->get_translation();
    const Vec3_t cam_center = keyfrm->get_cam_center();

    for (const auto lm : landmarks_to_check) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        if (lm->is_observed_in_keyframe(keyfrm)) {
            continue;
        }

        // 3D point coordinates with the global reference
        const Vec3_t pos_w = lm->get_pos_in_world();

        // Reproject and compute visibility
        Vec2_t reproj;
        float x_right;
        const bool in_image = keyfrm->camera_->reproject_to_image(rot_cw, trans_cw, pos_w, reproj, x_right);

        // Ignore if it is reprojected outside the image
        if (!in_image) {
            continue;
        }

        // Check if it's within ORB scale levels
        const Vec3_t cam_to_lm_vec = pos_w - cam_center;
        const auto cam_to_lm_dist = cam_to_lm_vec.norm();
        const auto max_cam_to_lm_dist = lm->get_max_valid_distance();
        const auto min_cam_to_lm_dist = lm->get_min_valid_distance();

        if (cam_to_lm_dist < min_cam_to_lm_dist || max_cam_to_lm_dist < cam_to_lm_dist) {
            continue;
        }

        // Compute the angle formed by the average vector of the 3D point observation,
        // and discard it if it is wider than the threshold value (60 degrees)
        const Vec3_t obs_mean_normal = lm->get_obs_mean_normal();

        if (cam_to_lm_vec.dot(obs_mean_normal) < 0.5 * cam_to_lm_dist) {
            continue;
        }

        // Acquire keypoints in the cell where the reprojected 3D points exist
        const auto pred_scale_level = lm->predict_scale_level(cam_to_lm_dist, keyfrm);
        const auto indices = keyfrm->get_keypoints_in_cell(reproj(0), reproj(1), margin * keyfrm->scale_factors_.at(pred_scale_level));

        if (indices.empty()) {
            continue;
        }

        // Find a keypoint with the closest descriptor
        const auto lm_desc = lm->get_descriptor();

        unsigned int best_dist = MAX_HAMMING_DIST;
        int best_idx = -1;

        for (const auto idx : indices) {
            const auto& keypt = keyfrm->undist_keypts_.at(idx);

            const auto scale_level = static_cast<unsigned int>(keypt.octave);

            // TODO: should determine the scale with 'keyfrm-> get_keypts_in_cell ()'
            if (scale_level < pred_scale_level - 1 || pred_scale_level < scale_level) {
                continue;
            }

            if (keyfrm->stereo_x_right_.at(idx) >= 0) {
                // Compute reprojection error with 3 degrees of freedom if a stereo match exists
                const auto e_x = reproj(0) - keypt.pt.x;
                const auto e_y = reproj(1) - keypt.pt.y;
                const auto e_x_right = x_right - keyfrm->stereo_x_right_.at(idx);
                const auto reproj_error_sq = e_x * e_x + e_y * e_y + e_x_right * e_x_right;

                // n=3
                constexpr float chi_sq_3D = 7.81473;
                if (chi_sq_3D < reproj_error_sq * keyfrm->inv_level_sigma_sq_.at(scale_level)) {
                    continue;
                }
            }
            else {
                // Compute reprojection error with 2 degrees of freedom if a stereo match does not exist
                const auto e_x = reproj(0) - keypt.pt.x;
                const auto e_y = reproj(1) - keypt.pt.y;
                const auto reproj_error_sq = e_x * e_x + e_y * e_y;

                // n=2
                constexpr float chi_sq_2D = 5.99146;
                if (chi_sq_2D < reproj_error_sq * keyfrm->inv_level_sigma_sq_.at(scale_level)) {
                    continue;
                }
            }

            const auto& desc = keyfrm->descriptors_.row(idx);

            const auto hamm_dist = compute_descriptor_distance_32(lm_desc, desc);

            if (hamm_dist < best_dist) {
                best_dist = hamm_dist;
                best_idx = idx;
            }
        }

        if (HAMMING_DIST_THR_LOW < best_dist) {
            continue;
        }

        auto lm_in_keyfrm = keyfrm->get_landmark(best_idx);
        if (lm_in_keyfrm) {
            // There is association between the 3D point and the keyframe
            // -> Duplication exists
            if (!lm_in_keyfrm->will_be_erased()) {
                // Replace with more reliable 3D points (= more observable)
                if (lm->num_observations() < lm_in_keyfrm->num_observations()) {
                    // Replace lm with lm_in_keyfrm
                    lm->replace(lm_in_keyfrm);
                }
                else {
                    // Replace lm_in_keyfrm with lm
                    lm_in_keyfrm->replace(lm);
                }
            }
        }
        else {
            // There is no association between the 3D point and the keyframe
            // Add the observation information
            lm->add_observation(keyfrm, best_idx);
            keyfrm->add_landmark(lm, best_idx);
        }

        ++num_fused;
    }

    return num_fused;
}

template unsigned int fuse::replace_duplication(data::keyframe*, const std::vector<data::landmark*>&, const float);
template unsigned int fuse::replace_duplication(data::keyframe*, const std::unordered_set<data::landmark*>&, const float);

} // namespace match
} // namespace openvslam
