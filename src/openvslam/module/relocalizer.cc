#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/module/relocalizer.h"
#include "openvslam/util/fancy_index.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace module {

relocalizer::relocalizer(data::bow_database* bow_db,
                         const double bow_match_lowe_ratio, const double proj_match_lowe_ratio,
                         const unsigned int min_num_bow_matches, const unsigned int min_num_valid_obs)
    : bow_db_(bow_db),
      min_num_bow_matches_(min_num_bow_matches), min_num_valid_obs_(min_num_valid_obs),
      bow_matcher_(bow_match_lowe_ratio, true), proj_matcher_(proj_match_lowe_ratio, true),
      pose_optimizer_() {
    spdlog::debug("CONSTRUCT: module::relocalizer");
}

relocalizer::~relocalizer() {
    spdlog::debug("DESTRUCT: module::relocalizer");
}

bool relocalizer::relocalize(data::frame& curr_frm) {
    curr_frm.compute_bow();

    // Acquire relocalization candidates
    const auto reloc_candidates = bow_db_->acquire_relocalization_candidates(&curr_frm);
    if (reloc_candidates.empty()) {
        return false;
    }
    const auto num_candidates = reloc_candidates.size();

    std::vector<std::vector<data::landmark*>> matched_landmarks(num_candidates);

    // Compute matching points for each candidate by using BoW tree matcher
    for (unsigned int i = 0; i < num_candidates; ++i) {
        auto keyfrm = reloc_candidates.at(i);
        if (keyfrm->will_be_erased()) {
            continue;
        }

        const auto num_matches = bow_matcher_.match_frame_and_keyframe(keyfrm, curr_frm, matched_landmarks.at(i));
        // Discard the candidate if the number of 2D-3D matches is less than the threshold
        if (num_matches < min_num_bow_matches_) {
            continue;
        }

        // Setup an PnP solver with the current 2D-3D matches
        const auto valid_indices = extract_valid_indices(matched_landmarks.at(i));
        auto pnp_solver = setup_pnp_solver(valid_indices, curr_frm.bearings_, curr_frm.keypts_,
                                           matched_landmarks.at(i), curr_frm.scale_factors_);

        // 1. Estimate the camera pose using EPnP (+ RANSAC)

        pnp_solver->find_via_ransac(30);
        if (!pnp_solver->solution_is_valid()) {
            continue;
        }

        curr_frm.cam_pose_cw_ = pnp_solver->get_best_cam_pose();
        curr_frm.update_pose_params();

        // 2. Apply pose optimizer

        // Get the inlier indices after EPnP+RANSAC
        const auto inlier_indices = util::resample_by_indices(valid_indices, pnp_solver->get_inlier_flags());

        // Set 2D-3D matches for the pose optimization
        curr_frm.landmarks_ = std::vector<data::landmark*>(curr_frm.num_keypts_, nullptr);
        std::set<data::landmark*> already_found_landmarks;
        for (const auto idx : inlier_indices) {
            // Set only the valid 3D points to the current frame
            curr_frm.landmarks_.at(idx) = matched_landmarks.at(i).at(idx);
            // Record the 3D points already associated to the frame keypoints
            already_found_landmarks.insert(matched_landmarks.at(i).at(idx));
        }

        // Pose optimization
        auto num_valid_obs = pose_optimizer_.optimize(curr_frm);
        // Discard the candidate if the number of the inliers is less than the threshold
        if (num_valid_obs < min_num_bow_matches_ / 2) {
            continue;
        }

        // Reject outliers
        for (unsigned int idx = 0; idx < curr_frm.num_keypts_; idx++) {
            if (!curr_frm.outlier_flags_.at(idx)) {
                continue;
            }
            curr_frm.landmarks_.at(idx) = nullptr;
        }

        // 3. Apply projection match to increase 2D-3D matches

        // Projection match based on the pre-optimized camera pose
        auto num_found = proj_matcher_.match_frame_and_keyframe(curr_frm, reloc_candidates.at(i), already_found_landmarks, 10, 100);
        // Discard the candidate if the number of the inliers is less than the threshold
        if (num_valid_obs + num_found < min_num_valid_obs_) {
            continue;
        }

        // 4. Re-apply the pose optimizer

        num_valid_obs = pose_optimizer_.optimize(curr_frm);

        // Apply projection match again if the number of the observations is less than the threshold
        if (num_valid_obs < min_num_valid_obs_) {
            // Exclude the already-associated landmarks
            already_found_landmarks.clear();
            for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
                if (!curr_frm.landmarks_.at(idx)) {
                    continue;
                }
                already_found_landmarks.insert(curr_frm.landmarks_.at(idx));
            }
            // Apply projection match again, then set the 2D-3D matches
            auto num_additional = proj_matcher_.match_frame_and_keyframe(curr_frm, reloc_candidates.at(i), already_found_landmarks, 3, 64);

            // Discard if the number of the observations is less than the threshold
            if (num_valid_obs + num_additional < min_num_valid_obs_) {
                continue;
            }

            // Perform optimization again
            num_valid_obs = pose_optimizer_.optimize(curr_frm);

            // Discard if falling below the threshold
            if (num_valid_obs < min_num_valid_obs_) {
                continue;
            }
        }

        // Succeeded in relocatization
        spdlog::info("relocalization succeeded");
        // TODO: should set the reference keyframe of the current frame

        // Reject outliers
        for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
            if (!curr_frm.outlier_flags_.at(idx)) {
                continue;
            }
            curr_frm.landmarks_.at(idx) = nullptr;
        }

        return true;
    }

    curr_frm.cam_pose_cw_is_valid_ = false;
    return false;
}

std::vector<unsigned int> relocalizer::extract_valid_indices(const std::vector<data::landmark*>& landmarks) const {
    std::vector<unsigned int> valid_indices;
    valid_indices.reserve(landmarks.size());
    for (unsigned int idx = 0; idx < landmarks.size(); ++idx) {
        auto lm = landmarks.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        valid_indices.push_back(idx);
    }
    return valid_indices;
}

std::unique_ptr<solve::pnp_solver> relocalizer::setup_pnp_solver(const std::vector<unsigned int>& valid_indices,
                                                                 const eigen_alloc_vector<Vec3_t>& bearings,
                                                                 const std::vector<cv::KeyPoint>& keypts,
                                                                 const std::vector<data::landmark*>& matched_landmarks,
                                                                 const std::vector<float>& scale_factors) const {
    // Resample valid elements
    const auto valid_bearings = util::resample_by_indices(bearings, valid_indices);
    const auto valid_keypts = util::resample_by_indices(keypts, valid_indices);
    const auto valid_assoc_lms = util::resample_by_indices(matched_landmarks, valid_indices);
    eigen_alloc_vector<Vec3_t> valid_landmarks(valid_indices.size());
    for (unsigned int i = 0; i < valid_indices.size(); ++i) {
        valid_landmarks.at(i) = valid_assoc_lms.at(i)->get_pos_in_world();
    }
    // Setup PnP solver
    return std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(valid_bearings, valid_keypts, valid_landmarks, scale_factors));
}

} // namespace module
} // namespace openvslam
