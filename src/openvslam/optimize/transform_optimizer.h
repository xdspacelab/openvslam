#ifndef OPENVSLAM_OPTIMIZE_SIM3_OPTIMIZER_H
#define OPENVSLAM_OPTIMIZE_SIM3_OPTIMIZER_H

#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <vector>

namespace openvslam {

namespace data {
class keyframe;
class landmark;
} // namespace data

namespace optimize {

class transform_optimizer {
public:
    /**
     * Constructor
     * @param fix_scale
     * @param num_iter
     */
    explicit transform_optimizer(const bool fix_scale, const unsigned int num_iter = 10);

    /**
     * Destructor
     */
    virtual ~transform_optimizer() = default;

    /**
     * Perform optimization
     * @param keyfrm_1
     * @param keyfrm_2
     * @param matched_lms_in_keyfrm_2
     * @param g2o_Sim3_12
     * @param chi_sq
     * @return
     */
    unsigned int optimize(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2,
                          std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_keyfrm_2,
                          g2o::Sim3& g2o_Sim3_12, const float chi_sq) const;

private:
    //! transform is Sim3 or SE3
    const bool fix_scale_;

    //! number of iterations of optimization
    const unsigned int num_iter_;
};

} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_SIM3_OPTIMIZER_H
