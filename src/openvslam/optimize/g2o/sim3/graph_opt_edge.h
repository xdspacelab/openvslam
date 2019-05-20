#ifndef OPENVSLAM_OPTIMIZER_G2O_SIM3_GRAPH_OPT_EDGE_H
#define OPENVSLAM_OPTIMIZER_G2O_SIM3_GRAPH_OPT_EDGE_H

#include "openvslam/type.h"
#include "openvslam/optimize/g2o/sim3/shot_vertex.h"

#include <g2o/core/base_binary_edge.h>

namespace openvslam {
namespace optimize {
namespace g2o {
namespace sim3 {

class graph_opt_edge final : public ::g2o::BaseBinaryEdge<7, ::g2o::Sim3, shot_vertex, shot_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    graph_opt_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override {
        const auto v1 = static_cast<const shot_vertex*>(_vertices.at(0));
        const auto v2 = static_cast<const shot_vertex*>(_vertices.at(1));

        const ::g2o::Sim3 C(_measurement);
        const ::g2o::Sim3 error_ = C * v1->estimate() * v2->estimate().inverse();
        _error = error_.log();
    }

    double initialEstimatePossible(const ::g2o::OptimizableGraph::VertexSet&, ::g2o::OptimizableGraph::Vertex*) override {
        return 1.0;
    }

    void initialEstimate(const ::g2o::OptimizableGraph::VertexSet& from, ::g2o::OptimizableGraph::Vertex*) override {
        auto v1 = static_cast<shot_vertex*>(_vertices[0]);
        auto v2 = static_cast<shot_vertex*>(_vertices[1]);
        if (0 < from.count(v1)) {
            v2->setEstimate(measurement() * v1->estimate());
        }
        else {
            v1->setEstimate(measurement().inverse() * v2->estimate());
        }
    }
};

} // namespace sim3
} // namespace g2o
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZER_G2O_SIM3_GRAPH_OPT_EDGE_H
