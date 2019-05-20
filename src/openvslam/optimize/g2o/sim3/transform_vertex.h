#ifndef OPENVSLAM_OPTIMIZE_G2O_SIM3_TRANSFORM_VERTEX_H
#define OPENVSLAM_OPTIMIZE_G2O_SIM3_TRANSFORM_VERTEX_H

#include "openvslam/type.h"

#include <g2o/core/base_vertex.h>
#include <g2o/types/sim3/sim3.h>

namespace openvslam {
namespace optimize {
namespace g2o {
namespace sim3 {

class transform_vertex final : public ::g2o::BaseVertex<7, ::g2o::Sim3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    transform_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override {
        _estimate = ::g2o::Sim3();
    }

    void oplusImpl(const number_t* update_) override {
        Eigen::Map<Vec7_t> update(const_cast<number_t*>(update_));

        if (fix_scale_) {
            update(6) = 0;
        }

        ::g2o::Sim3 s(update);
        setEstimate(s * estimate());
    }

    bool fix_scale_;

    Mat33_t rot_1w_;
    Vec3_t trans_1w_;
    Mat33_t rot_2w_;
    Vec3_t trans_2w_;
};

} // namespace sim3
} // namespace g2o
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_G2O_SIM3_TRANSFORM_VERTEX_H
