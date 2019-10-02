#include "openvslam/optimize/g2o/sim3/backward_reproj_edge.h"

namespace openvslam {
namespace optimize {
namespace g2o {
namespace sim3 {

base_backward_reproj_edge::base_backward_reproj_edge()
    : ::g2o::BaseUnaryEdge<2, Vec2_t, transform_vertex>() {}

bool base_backward_reproj_edge::read(std::istream& is) {
    for (unsigned int i = 0; i < 2; ++i) {
        is >> _measurement(i);
    }
    for (unsigned int i = 0; i < 2; ++i) {
        for (unsigned int j = i; j < 2; ++j) {
            is >> information()(i, j);
            if (i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }
    return true;
}

bool base_backward_reproj_edge::write(std::ostream& os) const {
    for (unsigned int i = 0; i < 2; ++i) {
        os << measurement()(i) << " ";
    }
    for (unsigned int i = 0; i < 2; ++i) {
        for (unsigned int j = i; j < 2; ++j) {
            os << " " << information()(i, j);
        }
    }
    return os.good();
}

perspective_backward_reproj_edge::perspective_backward_reproj_edge()
    : base_backward_reproj_edge() {}

equirectangular_backward_reproj_edge::equirectangular_backward_reproj_edge()
    : base_backward_reproj_edge() {}

} // namespace sim3
} // namespace g2o
} // namespace optimize
} // namespace openvslam
