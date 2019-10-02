#include "openvslam/optimize/g2o/sim3/graph_opt_edge.h"

namespace openvslam {
namespace optimize {
namespace g2o {
namespace sim3 {

graph_opt_edge::graph_opt_edge()
    : ::g2o::BaseBinaryEdge<7, ::g2o::Sim3, shot_vertex, shot_vertex>() {}

bool graph_opt_edge::read(std::istream& is) {
    Vec7_t sim3_wc;
    for (unsigned int i = 0; i < 7; ++i) {
        is >> sim3_wc(i);
    }
    const ::g2o::Sim3 Sim3_wc(sim3_wc);
    setMeasurement(Sim3_wc.inverse());
    for (unsigned int i = 0; i < 7; ++i) {
        for (unsigned int j = i; j < 7; ++j) {
            is >> information()(i, j);
            if (i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }
    return true;
}

bool graph_opt_edge::write(std::ostream& os) const {
    ::g2o::Sim3 Sim3_wc(measurement().inverse());
    const auto sim3_wc = Sim3_wc.log();
    for (unsigned int i = 0; i < 7; ++i) {
        os << sim3_wc(i) << " ";
    }
    for (unsigned int i = 0; i < 7; ++i) {
        for (unsigned int j = i; j < 7; ++j) {
            os << " " << information()(i, j);
        }
    }
    return os.good();
}

} // namespace sim3
} // namespace g2o
} // namespace optimize
} // namespace openvslam
