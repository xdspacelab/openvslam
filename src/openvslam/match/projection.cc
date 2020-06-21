#include "openvslam/camera/base.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/match/projection.h"
#include "openvslam/match/angle_checker.h"

namespace openvslam {
namespace match {

unsigned int projection::match_frame_and_landmarks(data::frame& frm, const std::vector<data::landmark*>& local_landmarks, const float margin) const {
    unsigned int num_matches = 0;

    // Reproject the 3D points to the frame, then acquire the 2D-3D matches
    for (auto local_lm : local_landmarks) {
        if (!local_lm->is_observable_in_tracking_) {
            continue;
        }
        if (local_lm->will_be_erased()) {
            continue;
        }

        const auto pred_scale_level = local_lm->scale_level_in_tracking_;

        // Acquire keypoints in the cell where the reprojected 3D points exist
        const auto indices_in_cell = frm.get_keypoints_in_cell(local_lm->reproj_in_tracking_(0), local_lm->reproj_in_tracking_(1),
                                                               margin * frm.scale_factors_.at(pred_scale_level),
                                                               pred_scale_level - 1, pred_scale_level);
        if (indices_in_cell.empty()) {
            continue;
        }

        const cv::Mat lm_desc = local_lm->get_descriptor();

        unsigned int best_hamm_dist = MAX_HAMMING_DIST;
        int best_scale_level = -1;
        unsigned int second_best_hamm_dist = MAX_HAMMING_DIST;
        int second_best_scale_level = -1;
        int best_idx = -1;

        for (const auto idx : indices_in_cell) {
            if (frm.landmarks_.at(idx) && frm.landmarks_.at(idx)->has_observation()) {
                continue;
            }

            if (0 < frm.stereo_x_right_.at(idx)) {
                const auto reproj_error = std::abs(local_lm->x_right_in_tracking_ - frm.stereo_x_right_.at(idx));
                if (margin * frm.scale_factors_.at(pred_scale_level) < reproj_error) {
                    continue;
                }
            }

            const cv::Mat& desc = frm.descriptors_.row(idx);

            const auto dist = compute_descriptor_distance_32(lm_desc, desc);

            if (dist < best_hamm_dist) {
                second_best_hamm_dist = best_hamm_dist;
                best_hamm_dist = dist;
                second_best_scale_level = best_scale_level;
                best_scale_level = frm.undist_keypts_.at(idx).octave;
                best_idx = idx;
            }
            else if (dist < second_best_hamm_dist) {
                second_best_scale_level = frm.undist_keypts_.at(idx).octave;
                second_best_hamm_dist = dist;
            }
        }

        if (best_hamm_dist <= HAMMING_DIST_THR_HIGH) {
            // Lowe's ratio test
            if (best_scale_level == second_best_scale_level && best_hamm_dist > lowe_ratio_ * second_best_hamm_dist) {
                continue;
            }

            // Add the matching information
            frm.landmarks_.at(best_idx) = local_lm;
            ++num_matches;
        }
    }

    return num_matches;
}

unsigned int projection::match_current_and_last_frames(data::frame& curr_frm, const data::frame& last_frm, const float margin) const {
    unsigned int num_matches = 0;

    angle_checker<int> angle_checker;

    const Mat33_t rot_cw = curr_frm.cam_pose_cw_.block<3, 3>(0, 0);
    const Vec3_t trans_cw = curr_frm.cam_pose_cw_.block<3, 1>(0, 3);

    const Vec3_t trans_wc = -rot_cw.transpose() * trans_cw;

    const Mat33_t rot_lw = last_frm.cam_pose_cw_.block<3, 3>(0, 0);
    const Vec3_t trans_lw = last_frm.cam_pose_cw_.block<3, 1>(0, 3);

    const Vec3_t trans_lc = rot_lw * trans_wc + trans_lw;

    // For non-monocular, check if the z component of the current-to-last translation vector is moving forward
    // The z component is positive going -> moving forward
    const bool assume_forward = (curr_frm.camera_->setup_type_ == camera::setup_type_t::Monocular)
                                    ? false
                                    : trans_lc(2) > curr_frm.camera_->true_baseline_;
    // The z component is negative going -> moving backward
    const bool assume_backward = (curr_frm.camera_->setup_type_ == camera::setup_type_t::Monocular)
                                     ? false
                                     : -trans_lc(2) > curr_frm.camera_->true_baseline_;

    // Reproject the 3D points associated to the keypoints of the last frame,
    // then acquire the 2D-3D matches
    for (unsigned int idx_last = 0; idx_last < last_frm.num_keypts_; ++idx_last) {
        auto lm = last_frm.landmarks_.at(idx_last);
        if (!lm) {
            continue;
        }
        // Discard any matches that were marked as outliers after applyng pose optimization
        if (last_frm.outlier_flags_.at(idx_last)) {
            continue;
        }

        // 3D point coordinates with the global reference
        const Vec3_t pos_w = lm->get_pos_in_world();

        // Reproject and compute visibility
        Vec2_t reproj;
        float x_right;
        const bool in_image = curr_frm.camera_->reproject_to_image(rot_cw, trans_cw, pos_w, reproj, x_right);

        // Ignore if it is reprojected outside the image
        if (!in_image) {
            continue;
        }

        // Acquire keypoints in the cell where the reprojected 3D points exist
        const auto last_scale_level = last_frm.keypts_.at(idx_last).octave;
        std::vector<unsigned int> indices;
        if (assume_forward) {
            indices = curr_frm.get_keypoints_in_cell(reproj(0), reproj(1),
                                                     margin * curr_frm.scale_factors_.at(last_scale_level),
                                                     last_scale_level, last_frm.num_scale_levels_ - 1);
        }
        else if (assume_backward) {
            indices = curr_frm.get_keypoints_in_cell(reproj(0), reproj(1),
                                                     margin * curr_frm.scale_factors_.at(last_scale_level),
                                                     0, last_scale_level);
        }
        else {
            indices = curr_frm.get_keypoints_in_cell(reproj(0), reproj(1),
                                                     margin * curr_frm.scale_factors_.at(last_scale_level),
                                                     last_scale_level - 1, last_scale_level + 1);
        }
        if (indices.empty()) {
            continue;
        }

        const auto lm_desc = lm->get_descriptor();

        unsigned int best_hamm_dist = MAX_HAMMING_DIST;
        int best_idx = -1;

        for (const auto curr_idx : indices) {
            if (curr_frm.landmarks_.at(curr_idx) && curr_frm.landmarks_[curr_idx]->has_observation()) {
                continue;
            }

            if (curr_frm.stereo_x_right_.at(curr_idx) > 0) {
                const float reproj_error = std::fabs(x_right - curr_frm.stereo_x_right_.at(curr_idx));
                if (margin * curr_frm.scale_factors_.at(last_scale_level) < reproj_error) {
                    continue;
                }
            }

            const auto& desc = curr_frm.descriptors_.row(curr_idx);

            const auto hamm_dist = compute_descriptor_distance_32(lm_desc, desc);

            if (hamm_dist < best_hamm_dist) {
                best_hamm_dist = hamm_dist;
                best_idx = curr_idx;
            }
        }

        if (HAMMING_DIST_THR_HIGH < best_hamm_dist) {
            continue;
        }

        // The matching is valid
        curr_frm.landmarks_.at(best_idx) = lm;
        ++num_matches;

        if (check_orientation_) {
            const auto delta_angle
                = last_frm.undist_keypts_.at(idx_last).angle - curr_frm.undist_keypts_.at(best_idx).angle;
            angle_checker.append_delta_angle(delta_angle, best_idx);
        }
    }

    if (check_orientation_) {
        const auto invalid_matches = angle_checker.get_invalid_matches();
        for (const auto invalid_idx : invalid_matches) {
            curr_frm.landmarks_.at(invalid_idx) = nullptr;
            --num_matches;
        }
    }

    return num_matches;
}

unsigned int projection::match_frame_and_keyframe(data::frame& curr_frm, data::keyframe* keyfrm, const std::set<data::landmark*>& already_matched_lms,
                                                  const float margin, const unsigned int hamm_dist_thr) const {
    unsigned int num_matches = 0;

    angle_checker<int> angle_checker;

    const Mat33_t rot_cw = curr_frm.cam_pose_cw_.block<3, 3>(0, 0);
    const Vec3_t trans_cw = curr_frm.cam_pose_cw_.block<3, 1>(0, 3);
    const Vec3_t cam_center = -rot_cw.transpose() * trans_cw;

    const auto landmarks = keyfrm->get_landmarks();

    // Reproject the 3D points associated to the keypoints of the keyframe,
    // then acquire the 2D-3D matches
    for (unsigned int idx = 0; idx < landmarks.size(); idx++) {
        auto lm = landmarks.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        // Avoid duplication
        if (already_matched_lms.count(lm)) {
            continue;
        }

        // 3D point coordinates with the global reference
        const Vec3_t pos_w = lm->get_pos_in_world();

        // Reproject and compute visibility
        Vec2_t reproj;
        float x_right;
        const bool in_image = curr_frm.camera_->reproject_to_image(rot_cw, trans_cw, pos_w, reproj, x_right);

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

        // Acquire keypoints in the cell where the reprojected 3D points exist
        const auto pred_scale_level = lm->predict_scale_level(cam_to_lm_dist, &curr_frm);

        const auto indices = curr_frm.get_keypoints_in_cell(reproj(0), reproj(1),
                                                            margin * curr_frm.scale_factors_.at(pred_scale_level),
                                                            pred_scale_level - 1, pred_scale_level + 1);

        if (indices.empty()) {
            continue;
        }

        const auto lm_desc = lm->get_descriptor();

        unsigned int best_hamm_dist = MAX_HAMMING_DIST;
        int best_idx = -1;

        for (unsigned long curr_idx : indices) {
            if (curr_frm.landmarks_.at(curr_idx)) {
                continue;
            }

            const auto& desc = curr_frm.descriptors_.row(curr_idx);

            const auto hamm_dist = compute_descriptor_distance_32(lm_desc, desc);

            if (hamm_dist < best_hamm_dist) {
                best_hamm_dist = hamm_dist;
                best_idx = curr_idx;
            }
        }

        if (hamm_dist_thr < best_hamm_dist) {
            continue;
        }

        // The matching is valid
        curr_frm.landmarks_.at(best_idx) = lm;
        num_matches++;

        if (check_orientation_) {
            const auto delta_angle
                = keyfrm->undist_keypts_.at(idx).angle - curr_frm.undist_keypts_.at(best_idx).angle;
            angle_checker.append_delta_angle(delta_angle, best_idx);
        }
    }

    if (check_orientation_) {
        const auto invalid_matches = angle_checker.get_invalid_matches();
        for (const auto invalid_idx : invalid_matches) {
            curr_frm.landmarks_.at(invalid_idx) = nullptr;
            --num_matches;
        }
    }

    return num_matches;
}

unsigned int projection::match_by_Sim3_transform(data::keyframe* keyfrm, const Mat44_t& Sim3_cw, const std::vector<data::landmark*>& landmarks,
                                                 std::vector<data::landmark*>& matched_lms_in_keyfrm, const float margin) const {
    unsigned int num_matches = 0;

    // Convert Sim3 into SE3
    const Mat33_t s_rot_cw = Sim3_cw.block<3, 3>(0, 0);
    const auto s_cw = std::sqrt(s_rot_cw.block<1, 3>(0, 0).dot(s_rot_cw.block<1, 3>(0, 0)));
    const Mat33_t rot_cw = s_rot_cw / s_cw;
    const Vec3_t trans_cw = Sim3_cw.block<3, 1>(0, 3) / s_cw;
    const Vec3_t cam_center = -rot_cw.transpose() * trans_cw;

    std::set<data::landmark*> already_matched(matched_lms_in_keyfrm.begin(), matched_lms_in_keyfrm.end());
    already_matched.erase(static_cast<data::landmark*>(nullptr));

    // Reproject the 3D points to the keyframe, then acquire the 2D-3D matches
    for (auto lm : landmarks) {
        if (lm->will_be_erased()) {
            continue;
        }
        if (already_matched.count(lm)) {
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

        // Compute the angle formed by the average observation vector of the 3D points,
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

        // Find keypoints with the closest descriptor
        const auto lm_desc = lm->get_descriptor();

        unsigned int best_dist = MAX_HAMMING_DIST;
        int best_idx = -1;

        for (const auto idx : indices) {
            if (matched_lms_in_keyfrm.at(idx)) {
                continue;
            }

            const auto scale_level = static_cast<unsigned int>(keyfrm->keypts_.at(idx).octave);

            // TODO: should determine the scale with 'keyfrm-> get_keypts_in_cell ()'
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

        matched_lms_in_keyfrm.at(best_idx) = lm;
        ++num_matches;
    }

    return num_matches;
}

unsigned int projection::match_keyframes_mutually(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2, std::vector<data::landmark*>& matched_lms_in_keyfrm_1,
                                                  const float& s_12, const Mat33_t& rot_12, const Vec3_t& trans_12, const float margin) const {
    // The pose of keyframe 1
    const Mat33_t rot_1w = keyfrm_1->get_rotation();
    const Vec3_t trans_1w = keyfrm_1->get_translation();

    // The pose of keyframe 2
    const Mat33_t rot_2w = keyfrm_2->get_rotation();
    const Vec3_t trans_2w = keyfrm_2->get_translation();

    // Compute the similarity transformation between the keyframes 1 and 2
    const Mat33_t s_rot_12 = s_12 * rot_12;
    const Mat33_t s_rot_21 = (1.0 / s_12) * rot_12.transpose();
    const Vec3_t trans_21 = -s_rot_21 * trans_12;

    const auto landmarks_1 = keyfrm_1->get_landmarks();
    const auto landmarks_2 = keyfrm_2->get_landmarks();

    // Contain matching information if there are already matches between the keyframes 1 and 2
    std::vector<bool> is_already_matched_in_keyfrm_1(landmarks_1.size(), false);
    std::vector<bool> is_already_matched_in_keyfrm_2(landmarks_2.size(), false);

    for (unsigned int idx_1 = 0; idx_1 < landmarks_1.size(); ++idx_1) {
        auto lm = matched_lms_in_keyfrm_1.at(idx_1);
        if (!lm) {
            continue;
        }
        const auto idx_2 = lm->get_index_in_keyframe(keyfrm_2);
        if (0 <= idx_2 && idx_2 < static_cast<int>(landmarks_2.size())) {
            is_already_matched_in_keyfrm_1.at(idx_1) = true;
            is_already_matched_in_keyfrm_2.at(idx_2) = true;
        }
    }

    std::vector<int> matched_indices_2_in_keyfrm_1(landmarks_1.size(), -1);
    std::vector<int> matched_indices_1_in_keyfrm_2(landmarks_2.size(), -1);

    // Compute the similarity transformation from the 3D points observed in keyframe 1 to keyframe 2 coordinates,
    // then project the result, and search keypoint matches
    // (world origin -- SE3 -> keyframe 1 -- Sim3 --> keyframe 2)
    // s_rot_21 * (rot_1w * pos_w + trans_1w) + trans_21
    // = s_rot_21 * rot_1w * pos_w + s_rot_21 * trans_1w + trans_21
    {
        const Mat33_t s_rot_21w = s_rot_21 * rot_1w;
        const Vec3_t trans_21w = s_rot_21 * trans_1w + trans_21;
        for (unsigned int idx_1 = 0; idx_1 < landmarks_1.size(); ++idx_1) {
            auto lm = landmarks_1.at(idx_1);
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            if (is_already_matched_in_keyfrm_1.at(idx_1)) {
                continue;
            }

            // 3D point coordinates with the global reference
            const Vec3_t pos_w = lm->get_pos_in_world();
            const Vec3_t pos_2 = s_rot_21w * pos_w + trans_21w;

            // Reproject and compute visibility
            Vec2_t reproj;
            float x_right;
            const bool in_image = keyfrm_2->camera_->reproject_to_image(s_rot_21w, trans_21w, pos_w, reproj, x_right);

            // Ignore if it is reprojected outside the image
            if (!in_image) {
                continue;
            }

            // Check if it's within ORB scale levels
            const auto cam_to_lm_dist = pos_2.norm();
            const auto max_cam_to_lm_dist = lm->get_max_valid_distance();
            const auto min_cam_to_lm_dist = lm->get_min_valid_distance();

            if (cam_to_lm_dist < min_cam_to_lm_dist || max_cam_to_lm_dist < cam_to_lm_dist) {
                continue;
            }

            // Acquire keypoints in the cell where the reprojected 3D points exist
            const auto pred_scale_level = lm->predict_scale_level(cam_to_lm_dist, keyfrm_2);
            const auto indices = keyfrm_2->get_keypoints_in_cell(reproj(0), reproj(1), margin * keyfrm_2->scale_factors_.at(pred_scale_level));

            if (indices.empty()) {
                continue;
            }

            // Find a keypoint with the closest descriptor
            const auto lm_desc = lm->get_descriptor();

            unsigned int best_hamm_dist = MAX_HAMMING_DIST;
            int best_idx_2 = -1;

            for (const auto idx_2 : indices) {
                const auto scale_level = static_cast<unsigned int>(keyfrm_2->keypts_.at(idx_2).octave);

                // TODO: should determine the scale with 'keyfrm-> get_keypts_in_cell ()'
                if (scale_level < pred_scale_level - 1 || pred_scale_level < scale_level) {
                    continue;
                }

                const auto& desc = keyfrm_2->descriptors_.row(idx_2);

                const auto hamm_dist = compute_descriptor_distance_32(lm_desc, desc);

                if (hamm_dist < best_hamm_dist) {
                    best_hamm_dist = hamm_dist;
                    best_idx_2 = idx_2;
                }
            }

            if (best_hamm_dist <= HAMMING_DIST_THR_HIGH) {
                matched_indices_2_in_keyfrm_1.at(idx_1) = best_idx_2;
            }
        }
    }

    // Compute the similarity transformation from the 3D points observed in the current keyframe (keyframe 1) to the candidate keyframe (keyframe 2) coordinates, then project the result
    // earch keypoint matches
    // (world origin -- SE3 -> keyframe2 -- Sim3 --> keyframe1)
    // s_rot_12 * (rot_2w * pos_w + trans_2w) + trans_12
    // = s_rot_12 * rot_2w * pos_w + s_rot_12 * trans_2w + trans_12
    {
        const Mat33_t s_rot_12w = s_rot_12 * rot_2w;
        const Vec3_t trans_12w = s_rot_12 * trans_2w + trans_12;
        for (unsigned int idx_2 = 0; idx_2 < landmarks_2.size(); ++idx_2) {
            auto lm = landmarks_2.at(idx_2);
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            if (is_already_matched_in_keyfrm_2.at(idx_2)) {
                continue;
            }

            // 3D point coordinates with the global reference
            const Vec3_t pos_w = lm->get_pos_in_world();
            const Vec3_t pos_1 = s_rot_12w * pos_w + trans_12w;

            // Reproject and compute visibility
            Vec2_t reproj;
            float x_right;
            const bool in_image = keyfrm_2->camera_->reproject_to_image(s_rot_12w, trans_12w, pos_w, reproj, x_right);

            // Ignore if it is reprojected outside the image
            if (!in_image) {
                continue;
            }

            // Check if it's within ORB scale levels
            const auto cam_to_lm_dist = pos_1.norm();
            const auto max_cam_to_lm_dist = lm->get_max_valid_distance();
            const auto min_cam_to_lm_dist = lm->get_min_valid_distance();

            if (cam_to_lm_dist < min_cam_to_lm_dist || max_cam_to_lm_dist < cam_to_lm_dist) {
                continue;
            }

            // Acquire keypoints in the cell where the reprojected 3D points exist
            const auto pred_scale_level = lm->predict_scale_level(cam_to_lm_dist, keyfrm_1);

            const auto indices = keyfrm_1->get_keypoints_in_cell(reproj(0), reproj(1), margin * keyfrm_1->scale_factors_.at(pred_scale_level));

            if (indices.empty()) {
                continue;
            }

            // Find a keypoint with the closest descriptor
            const auto lm_desc = lm->get_descriptor();

            unsigned int best_hamm_dist = MAX_HAMMING_DIST;
            int best_idx_1 = -1;

            for (const auto idx_1 : indices) {
                const auto scale_level = static_cast<unsigned int>(keyfrm_1->keypts_.at(idx_1).octave);

                // TODO: should determine the scale with 'keyfrm-> get_keypts_in_cell ()'
                if (scale_level < pred_scale_level - 1 || pred_scale_level < scale_level) {
                    continue;
                }

                const auto& desc = keyfrm_1->descriptors_.row(idx_1);

                const auto hamm_dist = compute_descriptor_distance_32(lm_desc, desc);

                if (hamm_dist < best_hamm_dist) {
                    best_hamm_dist = hamm_dist;
                    best_idx_1 = idx_1;
                }
            }

            if (best_hamm_dist <= HAMMING_DIST_THR_HIGH) {
                matched_indices_1_in_keyfrm_2.at(idx_2) = best_idx_1;
            }
        }
    }

    // Record only the cross-matches
    unsigned int num_matches = 0;
    for (unsigned int i = 0; i < landmarks_1.size(); ++i) {
        const auto idx_2 = matched_indices_2_in_keyfrm_1.at(i);
        if (idx_2 < 0) {
            continue;
        }

        const auto idx_1 = matched_indices_1_in_keyfrm_2.at(idx_2);
        if (idx_1 == static_cast<int>(i)) {
            matched_lms_in_keyfrm_1.at(idx_1) = landmarks_2.at(idx_2);
            ++num_matches;
        }
    }

    return num_matches;
}

} // namespace match
} // namespace openvslam
