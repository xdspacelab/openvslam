#ifndef OPENVSLAM_MAP_LOOP_DETECTOR_H
#define OPENVSLAM_MAP_LOOP_DETECTOR_H

#include "openvslam/data/bow_vocabulary.h"
#include "openvslam/map/type.h"
#include "openvslam/optimize/transform_optimizer.h"

#include <atomic>

namespace openvslam {

namespace data {
class keyframe;
class bow_database;
} // namespace data

namespace map {

class loop_detector {
public:
    loop_detector(data::bow_database* bow_db, data::bow_vocabulary* bow_vocab, const bool fix_scale);

    /**
     * Detect loop candidates using BoW vocabulary
     */
    bool detect_loop_candidates(data::keyframe* keyfrm);

    /**
     * Validate loop candidates selected in detect_loop_candidate()
     */
    bool validate_candidates();

    data::keyframe* get_final_candidate_keyframe() const {
        return final_candidate_keyfrm_;
    }

    g2o::Sim3 get_g2o_Sim3_world_to_curr() const {
        return g2o_Sim3_world_to_curr_;
    }

    std::vector<data::landmark*> get_curr_assoc_lms_in_cand() const {
        return curr_assoc_lms_in_cand_;
    }

    std::vector<data::landmark*> get_curr_assoc_lms_near_cand() const {
        return curr_assoc_lms_near_cand_;
    }

    void set_prev_loop_correct_keyfrm_id(const unsigned int prev_loop_correct_keyfrm_id) {
        prev_loop_correct_keyfrm_id_ = prev_loop_correct_keyfrm_id;
    }

private:
    void enable_loop_detector() {
        loop_detector_is_enabled_ = true;
    }

    void disable_loop_detector() {
        loop_detector_is_enabled_ = false;
    }

    /**
     * Compute the minimum score among covisibilities
     */
    float compute_min_score_in_covisibilities(data::keyframe* keyfrm) const;

    /**
     * Find continuously detected keyframe sets
     */
    keyframe_sets find_continuously_detected_keyframe_sets(const keyframe_sets& prev_cont_detected_keyfrm_sets,
                                                           const std::vector<data::keyframe*>& keyfrms_to_search) const;

    /**
     * Select ONE candidate from the candidates via linear and nonlinear Sim3 estimation
     */
    bool select_loop_candidate_via_Sim3(const std::vector<data::keyframe*>& loop_candidates,
                                        data::keyframe*& selected_candidate,
                                        g2o::Sim3& g2o_Sim3_world_to_curr,
                                        std::vector<data::landmark*>& curr_assoc_lms_in_cand) const;

    data::bow_database* bow_db_;
    data::bow_vocabulary* bow_vocab_;

    //! transform optimizer
    const optimize::transform_optimizer transform_optimizer_;

    std::atomic<bool> loop_detector_is_enabled_{true};

    //-----------------------------------------
    // variables for loop detection and correction

    const bool fix_scale_;

    //! この回数以上連続してキーフレーム集合が検出されたらループとする
    static constexpr unsigned int min_continuity_ = 3;
    //! 現在処理中のキーフレーム
    data::keyframe* cur_keyfrm_;
    //! 最終的なループ候補
    data::keyframe* final_candidate_keyfrm_ = nullptr;

    //! 前回検出されたキーフレーム集合
    std::vector<keyframe_set> cont_detected_keyfrm_sets_;
    //! validate対象のキーフレーム
    std::vector<data::keyframe*> loop_candidates_to_validate_;

    //! current keyframeの特徴点idxに対する，candidateで観測している3次元との対応情報
    std::vector<data::landmark*> curr_assoc_lms_in_cand_;
    //! current keyframeの特徴点idxに対する，candidate周辺で観測している3次元との対応情報
    std::vector<data::landmark*> curr_assoc_lms_near_cand_;

    //! ループ補正後のcurrent keyframeの姿勢
    Mat44_t Sim3_world_to_curr_;
    //! ループ補正後のcurrent keyframeの姿勢(g2o::Sim3形式)
    g2o::Sim3 g2o_Sim3_world_to_curr_;

    //! 前回ループ修正をした際のcurrent keyframeのID
    unsigned int prev_loop_correct_keyfrm_id_ = 0;
};

} // namespace map
} // namespace openvslam

#endif // OPENVSLAM_MAP_LOOP_DETECTOR_H
