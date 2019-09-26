#include "openvslam/optimize/g2o/sim3/transform_vertex.h"

namespace openvslam {
namespace optimize {
namespace g2o {
namespace sim3 {

sim3::transform_vertex::transform_vertex()
    : ::g2o::BaseVertex<7, ::g2o::Sim3>() {}

bool transform_vertex::read(std::istream& is) {
    Vec7_t g2o_sim3_wc;
    for (int i = 0; i < 7; ++i) {
        is >> g2o_sim3_wc(i);
    }
    setEstimate(::g2o::Sim3(g2o_sim3_wc).inverse());
    return true;
}

bool transform_vertex::write(std::ostream& os) const {
    ::g2o::Sim3 g2o_Sim3_wc(estimate().inverse());
    const Vec7_t g2o_sim3_wc = g2o_Sim3_wc.log();
    for (int i = 0; i < 7; ++i) {
        os << g2o_sim3_wc(i) << " ";
    }
    return os.good();
}

} // namespace sim3
} // namespace g2o
} // namespace optimize
} // namespace openvslam