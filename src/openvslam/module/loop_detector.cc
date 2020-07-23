#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/match/bow_tree.h"
#include "openvslam/match/projection.h"
#include "openvslam/module/loop_detector.h"
#include "openvslam/solve/sim3_solver.h"
#include "openvslam/util/converter.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace module {

loop_detector::loop_detector(data::bow_database* bow_db, data::bow_vocabulary* bow_vocab, const bool fix_scale_in_Sim3_estimation)
    : bow_db_(bow_db), bow_vocab_(bow_vocab), transform_optimizer_(fix_scale_in_Sim3_estimation),
      fix_scale_in_Sim3_estimation_(fix_scale_in_Sim3_estimation) {}

void loop_detector::enable_loop_detector() {
    loop_detector_is_enabled_ = true;
}

void loop_detector::disable_loop_detector() {
    loop_detector_is_enabled_ = false;
}

bool loop_detector::is_enabled() const {
    return loop_detector_is_enabled_;
}

void loop_detector::set_current_keyframe(const std::shared_ptr<data::keyframe>& keyfrm) {
    cur_keyfrm_ = keyfrm;
}

bool loop_detector::detect_loop_candidates() {
    // if the loop detector is disabled or the loop has been corrected recently,
    // cannot perfrom the loop correction
    if (!loop_detector_is_enabled_ || cur_keyfrm_->id_ < prev_loop_correct_keyfrm_id_ + 10) {
        // register to the BoW database
        bow_db_->add_keyframe(cur_keyfrm_);
        return false;
    }

    // 1. search loop candidates by inquiring to the BoW dataqbase

    // 1-1. before inquiring, compute the minimum score of BoW similarity between the current and each of the covisibilities

    const float min_score = compute_min_score_in_covisibilities(cur_keyfrm_);

    // 1-2. inquiring to the BoW database about the similar keyframe whose score is lower than min_score

    const auto init_loop_candidates = bow_db_->acquire_loop_candidates(cur_keyfrm_, min_score);

    // 1-3. if no candidates are found, cannot perform the loop correction

    if (init_loop_candidates.empty()) {
        // clear the buffer because any candidates are not found
        cont_detected_keyfrm_sets_.clear();
        // register to the BoW database
        bow_db_->add_keyframe(cur_keyfrm_);
        return false;
    }

    // 2. From now on, we treat each of the candidates as "keyframe set" in order to improve robustness of loop detection
    //    the number of each of the candidate keyframe sets that detected are counted every time when this member functions is called
    //    if the keyframe sets were detected at the previous call, it is contained in `cont_detected_keyfrm_sets_`
    //    (note that "match of two keyframe sets" means the intersection of the two sets is NOT empty)

    const auto curr_cont_detected_keyfrm_sets = find_continuously_detected_keyframe_sets(cont_detected_keyfrm_sets_, init_loop_candidates);

    // 3. if the number of the detection is equal of greater than the threshold (`min_continuity_`),
    //    adopt it as one of the loop candidates

    loop_candidates_to_validate_.clear();
    for (auto& curr : curr_cont_detected_keyfrm_sets) {
        const auto candidate_keyfrm = curr.lead_keyfrm_;
        const auto continuity = curr.continuity_;
        // check if the number of the detection is equal of greater than the threshold
        if (min_continuity_ <= continuity) {
            // adopt as the candidates
            loop_candidates_to_validate_.push_back(candidate_keyfrm);
        }
    }

    // 4. Update the members for the next call of this function

    cont_detected_keyfrm_sets_ = curr_cont_detected_keyfrm_sets;

    // register to the BoW database
    bow_db_->add_keyframe(cur_keyfrm_);

    // return any candidate is found or not
    return !loop_candidates_to_validate_.empty();
}

bool loop_detector::validate_candidates() {
    // disallow the removal of the candidates
    for (const auto candidate : loop_candidates_to_validate_) {
        candidate->set_not_to_be_erased();
    }

    // 1. for each of the candidates, estimate and validate the Sim3 between it and the current keyframe using the observed landmarks
    //    then, select ONE candaite

    const bool candidate_is_found = select_loop_candidate_via_Sim3(loop_candidates_to_validate_, selected_candidate_,
                                                                   g2o_Sim3_world_to_curr_, curr_match_lms_observed_in_cand_);
    Sim3_world_to_curr_ = util::converter::to_eigen_mat(g2o_Sim3_world_to_curr_);

    if (!candidate_is_found) {
        for (const auto loop_candidate : loop_candidates_to_validate_) {
            loop_candidate->set_to_be_erased();
        }
        return false;
    }

    spdlog::debug("detect loop candidate via Sim3 estimation: keyframe {} - keyframe {}", selected_candidate_->id_, cur_keyfrm_->id_);

    // 2. reproject the landmarks observed in covisibilities of the selected candidate to the current keyframe,
    //    then acquire the extra 2D-3D matches

    // matches between the keypoints in the current and the landmarks observed in the covisibilities of the selected candidate
    curr_match_lms_observed_in_cand_covis_.clear();

    auto cand_covisibilities = selected_candidate_->graph_node_->get_covisibilities();
    cand_covisibilities.push_back(selected_candidate_);

    // acquire all of the landmarks observed in the covisibilities of the candidate
    // check the already inserted landmarks
    std::unordered_set<std::shared_ptr<data::landmark>> already_inserted;
    for (const auto covisibility : cand_covisibilities) {
        const auto lms_in_covisibility = covisibility->get_landmarks();
        for (const auto& lm : lms_in_covisibility) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            if (already_inserted.count(lm)) {
                continue;
            }
            curr_match_lms_observed_in_cand_covis_.push_back(lm);
            already_inserted.insert(lm);
        }
    }

    // reproject the landmarks observed in the covisibilities of the candidate to the current keyframe using Sim3 `Sim3_world_to_curr_`,
    // then, acquire the extra 2D-3D matches
    // however, landmarks in `curr_match_lms_observed_in_cand_` are already matched with keypoints in the current keyframe,
    // thus they are excluded from the reprojection
    match::projection projection_matcher(0.75, true);
    projection_matcher.match_by_Sim3_transform(cur_keyfrm_, Sim3_world_to_curr_, curr_match_lms_observed_in_cand_covis_,
                                               curr_match_lms_observed_in_cand_, 10);

    // count up the matches
    unsigned int num_final_matches = 0;
    for (const auto curr_assoc_lm_in_cand : curr_match_lms_observed_in_cand_) {
        if (curr_assoc_lm_in_cand) {
            ++num_final_matches;
        }
    }

    spdlog::debug("acquired {} matches after projection-match", num_final_matches);

    // if the number of matches is greater than the threshold, adopt the selected candidate for the loop correction
    constexpr unsigned int num_final_matches_thr = 40;
    if (num_final_matches_thr <= num_final_matches) {
        // allow the removal of the candidates except for the selected one
        for (const auto loop_candidate : loop_candidates_to_validate_) {
            if (*loop_candidate == *selected_candidate_) {
                continue;
            }
            loop_candidate->set_to_be_erased();
        }
        return true;
    }
    else {
        spdlog::debug("destruct loop candidate because enough matches not acquired (< {})", num_final_matches_thr);
        // allow the removal of all of the candidates
        for (const auto loop_candidate : loop_candidates_to_validate_) {
            loop_candidate->set_to_be_erased();
        }
        return false;
    }
}

float loop_detector::compute_min_score_in_covisibilities(const std::shared_ptr<data::keyframe>& keyfrm) const {
    // the maximum of score is 1.0
    float min_score = 1.0;

    // search the mininum score among covisibilities
    const auto covisibilities = keyfrm->graph_node_->get_covisibilities();
    const auto& bow_vec_1 = keyfrm->bow_vec_;
    for (const auto covisibility : covisibilities) {
        if (covisibility->will_be_erased()) {
            continue;
        }
        const auto& bow_vec_2 = covisibility->bow_vec_;

#ifdef USE_DBOW2
        const float score = bow_vocab_->score(bow_vec_1, bow_vec_2);
#else
        const float score = fbow::BoWVector::score(bow_vec_1, bow_vec_2);
#endif
        if (score < min_score) {
            min_score = score;
        }
    }

    return min_score;
}

keyframe_sets loop_detector::find_continuously_detected_keyframe_sets(const keyframe_sets& prev_cont_detected_keyfrm_sets,
                                                                      const std::vector<std::shared_ptr<data::keyframe>>& keyfrms_to_search) const {
    // count up the number of the detection of each of the keyframe sets

    // buffer to store continuity and keyframe set
    keyframe_sets curr_cont_detected_keyfrm_sets;

    // check the already counted keyframe sets to prevent from counting the same set twice
    std::map<std::set<std::shared_ptr<data::keyframe>>, bool> already_checked;
    for (const auto& prev : prev_cont_detected_keyfrm_sets) {
        already_checked[prev.keyfrm_set_] = false;
    }

    for (const auto& keyfrm_to_search : keyfrms_to_search) {
        // enlarge the candidate to the "keyframe set"
        const auto keyfrm_set = keyfrm_to_search->graph_node_->get_connected_keyframes();

        // check if the initialization of the buffer is needed or not
        bool initialization_is_needed = true;

        // check continuity for each of the previously detected keyframe set
        for (const auto& prev : prev_cont_detected_keyfrm_sets) {
            // prev.keyfrm_set_: keyframe set
            // prev.lead_keyfrm_: the leader keyframe of the set
            // prev.continuity_: continuity

            // check if the keyframe set is already counted or not
            if (already_checked.at(prev.keyfrm_set_)) {
                continue;
            }

            // compute intersection between the previous set and the current set, then check if it is empty or not
            if (prev.intersection_is_empty(keyfrm_set)) {
                continue;
            }

            // initialization is not needed because any candidate is found
            initialization_is_needed = false;

            // create the new statistics by incrementing the continuity
            const auto curr_continuity = prev.continuity_ + 1;
            curr_cont_detected_keyfrm_sets.emplace_back(
                keyframe_set{keyfrm_set, keyfrm_to_search, curr_continuity});

            // this keyframe set is already checked
            already_checked.at(prev.keyfrm_set_) = true;
        }

        // if initialization is needed, add the new statistics
        if (initialization_is_needed) {
            curr_cont_detected_keyfrm_sets.emplace_back(
                keyframe_set{keyfrm_set, keyfrm_to_search, 0});
        }
    }

    return curr_cont_detected_keyfrm_sets;
}

bool loop_detector::select_loop_candidate_via_Sim3(const std::vector<std::shared_ptr<data::keyframe>>& loop_candidates,
                                                   std::shared_ptr<data::keyframe>& selected_candidate,
                                                   g2o::Sim3& g2o_Sim3_world_to_curr,
                                                   std::vector<std::shared_ptr<data::landmark>>& curr_match_lms_observed_in_cand) const {
    // estimate and the Sim3 between the current keyframe and each of the candidates using the observed landmarks
    // the Sim3 is estimated both in linear and non-linear ways
    // if the inlier after the estimation is lower than the threshold, discard tha candidate

    match::bow_tree bow_matcher(0.75, true);
    match::projection projection_matcher(0.75, true);

    for (const auto& candidate : loop_candidates) {
        if (candidate->will_be_erased()) {
            continue;
        }

        // estimate the matches between the keypoints in the current keyframe and the landmarks observed in the candidate
        curr_match_lms_observed_in_cand.clear();
        const auto num_matches = bow_matcher.match_keyframes(cur_keyfrm_, candidate, curr_match_lms_observed_in_cand);

        // check the threshold
        if (num_matches < 20) {
            continue;
        }

        // estimate the Sim3 using keypoint-landmark matches in linear way
        // keyframe1: current keyframe, keyframe2: candidate keyframe
        // estimate Sim3 of 2->1 (candidate->current)

        solve::sim3_solver solver(cur_keyfrm_, candidate, curr_match_lms_observed_in_cand,
                                  fix_scale_in_Sim3_estimation_, 20);
        solver.find_via_ransac(200);
        if (!solver.solution_is_valid()) {
            continue;
        }

        spdlog::debug("found loop candidate via linear Sim3 estimation: keyframe {} - keyframe {}", candidate->id_, cur_keyfrm_->id_);

        // find additional matches by reprojection landmarks each other

        const Mat33_t rot_cand_to_curr = solver.get_best_rotation_12();
        const Vec3_t trans_cand_to_curr = solver.get_best_translation_12();
        const float scale_cand_to_curr = solver.get_best_scale_12();

        // perforn non-linear optimization of the estimated Sim3

        projection_matcher.match_keyframes_mutually(cur_keyfrm_, candidate, curr_match_lms_observed_in_cand,
                                                    scale_cand_to_curr, rot_cand_to_curr, trans_cand_to_curr, 7.5);

        g2o::Sim3 g2o_sim3_cand_to_curr(rot_cand_to_curr, trans_cand_to_curr, scale_cand_to_curr);
        const auto num_optimized_inliers = transform_optimizer_.optimize(cur_keyfrm_, candidate, curr_match_lms_observed_in_cand,
                                                                         g2o_sim3_cand_to_curr, 10);

        // check the threshold
        if (num_optimized_inliers < 20) {
            continue;
        }

        spdlog::debug("found loop candidate via nonlinear Sim3 optimization: keyframe {} - keyframe {}", candidate->id_, cur_keyfrm_->id_);

        selected_candidate = candidate;
        // convert the estimated Sim3 from "candidate -> current" to "world -> current"
        // this Sim3 indicates the correct camera pose oof the current keyframe after loop correction
        g2o_Sim3_world_to_curr = g2o_sim3_cand_to_curr * g2o::Sim3(candidate->get_rotation(), candidate->get_translation(), 1.0);

        return true;
    }

    return false;
}

std::shared_ptr<data::keyframe> loop_detector::get_selected_candidate_keyframe() const {
    return selected_candidate_;
}

g2o::Sim3 loop_detector::get_Sim3_world_to_current() const {
    return g2o_Sim3_world_to_curr_;
}

std::vector<std::shared_ptr<data::landmark>> loop_detector::current_matched_landmarks_observed_in_candidate() const {
    return curr_match_lms_observed_in_cand_;
}

std::vector<std::shared_ptr<data::landmark>> loop_detector::current_matched_landmarks_observed_in_candidate_covisibilities() const {
    return curr_match_lms_observed_in_cand_covis_;
}

void loop_detector::set_loop_correct_keyframe_id(const unsigned int loop_correct_keyfrm_id) {
    prev_loop_correct_keyfrm_id_ = loop_correct_keyfrm_id;
}

} // namespace module
} // namespace openvslam
