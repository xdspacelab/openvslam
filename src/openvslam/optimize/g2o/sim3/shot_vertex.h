#ifndef OPENVSLAM_OPTIMIZER_G2O_SIM3_SHOT_VERTEX_H
#define OPENVSLAM_OPTIMIZER_G2O_SIM3_SHOT_VERTEX_H

#include "openvslam/type.h"

#include <g2o/core/base_vertex.h>
#include <g2o/types/sim3/sim3.h>

namespace openvslam {
namespace optimize {
namespace g2o {
namespace sim3 {

class shot_vertex final : public ::g2o::BaseVertex<7, ::g2o::Sim3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    shot_vertex();

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

        const ::g2o::Sim3 s(update);
        setEstimate(s * estimate());
    }

    bool fix_scale_;
};

} // namespace sim3
} // namespace g2o
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZER_G2O_SIM3_SHOT_VERTEX_H
