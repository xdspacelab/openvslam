#ifndef OPENVSLAM_MATCH_BOW_TREE_H
#define OPENVSLAM_MATCH_BOW_TREE_H

#include "openvslam/match/base.h"

namespace openvslam {

namespace data {
class frame;
class keyframe;
class landmark;
} // namespace data

namespace match {

class bow_tree final : public base {
public:
    explicit bow_tree(const float lowe_ratio = 0.6, const bool check_orientation = true)
        : base(lowe_ratio, check_orientation) {}

    ~bow_tree() final = default;

    //! frameで観測している特徴点とkeyframeで観測している特徴点の対応を求め，それを元にframeの特徴点と3次元点の対応情報を得る
    //! matched_lms_in_frmには，frameの各特徴点に対応する(keyframeで観測された)3次元点が格納される
    //! NOTE: matched_lms_in_frm.size()はframeの特徴点数と一致
    unsigned int match_frame_and_keyframe(data::keyframe* keyfrm, data::frame& frm, std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_frm) const;

    //! keyframe1で観測している特徴点とkeyframe2で観測している特徴点の対応を求め，それを元にkeyframe1の特徴点と3次元点の対応情報を得る
    //! matched_lms_in_keyfrm_1には，keyframe1の各特徴点に対応する(keyframe2で観測された)3次元点が格納される
    //! NOTE: matched_lms_in_keyfrm_1.size()はkeyframe1の特徴点数と一致
    unsigned int match_keyframes(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2, std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_keyfrm_1) const;
};

} // namespace match
} // namespace openvslam

#endif // OPENVSLAM_MATCH_BOW_TREE_H
