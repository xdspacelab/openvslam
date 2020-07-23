#ifndef OPENVSLAM_MODULE_TYPE_H
#define OPENVSLAM_MODULE_TYPE_H

#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace openvslam {
namespace module {

// 相互参照を避けるためにここでtypedefしておく
typedef std::map<std::shared_ptr<data::keyframe>,
                 g2o::Sim3,
                 std::less<std::shared_ptr<data::keyframe>>,
                 Eigen::aligned_allocator<std::pair<std::shared_ptr<data::keyframe> const, g2o::Sim3>>>
    keyframe_Sim3_pairs_t;

// キーフレームの集合, 中心のキーフレーム, 連続検出回数を合わせた構造体
struct keyframe_set {
    keyframe_set(const std::set<std::shared_ptr<data::keyframe>>& keyfrm_set, const std::shared_ptr<data::keyframe>& lead_keyfrm, const unsigned int continuity)
        : keyfrm_set_(keyfrm_set), lead_keyfrm_(lead_keyfrm), continuity_(continuity) {}
    std::set<std::shared_ptr<data::keyframe>> keyfrm_set_;
    std::shared_ptr<data::keyframe> lead_keyfrm_;
    unsigned int continuity_ = 0;

    bool intersection_is_empty(const std::set<std::shared_ptr<data::keyframe>>& other_set) const {
        for (const auto& this_keyfrm : keyfrm_set_) {
            if (static_cast<bool>(other_set.count(this_keyfrm))) {
                return false;
            }
        }
        return true;
    }

    bool intersection_is_empty(const keyframe_set& other_set) const {
        return intersection_is_empty(other_set.keyfrm_set_);
    }
};

using keyframe_sets = eigen_alloc_vector<keyframe_set>;

} // namespace module
} // namespace openvslam

#endif // OPENVSLAM_MODULE_TYPE_H
