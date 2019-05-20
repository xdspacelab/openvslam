#ifndef OPENVSLAM_OPTIMIZER_G2O_SE3_SHOT_VERTEX_H
#define OPENVSLAM_OPTIMIZER_G2O_SE3_SHOT_VERTEX_H

#include "openvslam/type.h"

#include <g2o/core/base_vertex.h>
#include <g2o/types/slam3d/se3quat.h>

namespace openvslam {
namespace optimize {
namespace g2o {
namespace se3 {

class shot_vertex final : public ::g2o::BaseVertex<6, ::g2o::SE3Quat> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    shot_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override {
        _estimate = ::g2o::SE3Quat();
    }

    void oplusImpl(const number_t* update_) override {
        Eigen::Map<const Vec6_t> update(update_);
        setEstimate(::g2o::SE3Quat::exp(update) * estimate());
    }
};

} // namespace se3
} // namespace g2o
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZER_G2O_SE3_SHOT_VERTEX_H
