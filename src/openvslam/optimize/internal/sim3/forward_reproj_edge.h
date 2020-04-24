#ifndef OPENVSLAM_OPTIMIZE_G2O_FORWARD_REPROJ_EDGE_H
#define OPENVSLAM_OPTIMIZE_G2O_FORWARD_REPROJ_EDGE_H

#include "openvslam/type.h"
#include "openvslam/optimize/internal/sim3/transform_vertex.h"

#include <g2o/core/base_unary_edge.h>

namespace openvslam {
namespace optimize {
namespace internal {
namespace sim3 {

class base_forward_reproj_edge : public g2o::BaseUnaryEdge<2, Vec2_t, transform_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    base_forward_reproj_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() final;

    virtual Vec2_t cam_project(const Vec3_t& pos_c) const = 0;

    Vec3_t pos_w_;
};

inline base_forward_reproj_edge::base_forward_reproj_edge()
    : g2o::BaseUnaryEdge<2, Vec2_t, transform_vertex>() {}

inline bool base_forward_reproj_edge::read(std::istream& is) {
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

inline bool base_forward_reproj_edge::write(std::ostream& os) const {
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

inline void base_forward_reproj_edge::computeError() {
    // vertexからSim3(2->1)を取り出す
    const auto v1 = static_cast<const transform_vertex*>(_vertices.at(0));
    const g2o::Sim3& Sim3_12 = v1->estimate();
    // vertexからSE3(world->2)を取り出す
    const Mat33_t& rot_2w = v1->rot_2w_;
    const Vec3_t& trans_2w = v1->trans_2w_;

    // 点の座標系を変換(world->2)
    const Vec3_t pos_2 = rot_2w * pos_w_ + trans_2w;
    // Sim3を使ってさらに変換(2->1)
    const Vec3_t pos_1 = Sim3_12.map(pos_2);
    // 再投影誤差を計算
    const Vec2_t obs(_measurement);
    _error = obs - cam_project(pos_1);
}

class perspective_forward_reproj_edge final : public base_forward_reproj_edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    perspective_forward_reproj_edge();

    Vec2_t cam_project(const Vec3_t& pos_c) const override;

    double fx_, fy_, cx_, cy_;
};

inline perspective_forward_reproj_edge::perspective_forward_reproj_edge()
    : base_forward_reproj_edge() {}

inline Vec2_t perspective_forward_reproj_edge::cam_project(const Vec3_t& pos_c) const {
    return {fx_ * pos_c(0) / pos_c(2) + cx_, fy_ * pos_c(1) / pos_c(2) + cy_};
}

class equirectangular_forward_reproj_edge final : public base_forward_reproj_edge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    equirectangular_forward_reproj_edge();

    Vec2_t cam_project(const Vec3_t& pos_c) const override;

    double cols_, rows_;
};

inline equirectangular_forward_reproj_edge::equirectangular_forward_reproj_edge()
    : base_forward_reproj_edge() {}

inline Vec2_t equirectangular_forward_reproj_edge::cam_project(const Vec3_t& pos_c) const {
    const double theta = std::atan2(pos_c(0), pos_c(2));
    const double phi = -std::asin(pos_c(1) / pos_c.norm());
    return {cols_ * (0.5 + theta / (2 * M_PI)), rows_ * (0.5 - phi / M_PI)};
}

} // namespace sim3
} // namespace internal
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_G2O_FORWARD_REPROJ_EDGE_H
