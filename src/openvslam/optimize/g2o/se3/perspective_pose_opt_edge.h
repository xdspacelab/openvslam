#ifndef OPENVSLAM_OPTIMIZER_G2O_SE3_PERSPECTIVE_POSE_OPT_EDGE_H
#define OPENVSLAM_OPTIMIZER_G2O_SE3_PERSPECTIVE_POSE_OPT_EDGE_H

#include "openvslam/type.h"
#include "openvslam/optimize/g2o/landmark_vertex.h"
#include "openvslam/optimize/g2o/se3/shot_vertex.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>

namespace openvslam {
namespace optimize {
namespace g2o {
namespace se3 {

class mono_perspective_pose_opt_edge final : public ::g2o::BaseUnaryEdge<2, Vec2_t, shot_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    mono_perspective_pose_opt_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override {
        const auto v1 = static_cast<const shot_vertex*>(_vertices.at(0));
        const Vec2_t obs(_measurement);
        _error = obs - cam_project(v1->estimate().map(pos_w_));
    }

    void linearizeOplus() override;

    bool depth_is_positive() const {
        const auto v1 = static_cast<const shot_vertex*>(_vertices.at(0));
        return 0 < (v1->estimate().map(pos_w_))(2);
    }

    inline Vec2_t cam_project(const Vec3_t& pos_c) const {
        return {fx_ * pos_c(0) / pos_c(2) + cx_, fy_ * pos_c(1) / pos_c(2) + cy_};
    }

    Vec3_t pos_w_;
    number_t fx_, fy_, cx_, cy_;
};

class stereo_perspective_pose_opt_edge : public ::g2o::BaseUnaryEdge<3, Vec3_t, shot_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    stereo_perspective_pose_opt_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override {
        const auto v1 = static_cast<const shot_vertex*>(_vertices.at(0));
        const Vec3_t obs(_measurement);
        _error = obs - cam_project(v1->estimate().map(pos_w_));
    }

    void linearizeOplus() override;

    bool depth_is_positive() const {
        const auto v1 = static_cast<const shot_vertex*>(_vertices.at(0));
        return 0 < (v1->estimate().map(pos_w_))(2);
    }

    inline Vec3_t cam_project(const Vec3_t& pos_c) const {
        const double reproj_x = fx_ * pos_c(0) / pos_c(2) + cx_;
        return {reproj_x, fy_ * pos_c(1) / pos_c(2) + cy_, reproj_x - focal_x_baseline_ / pos_c(2)};
    }

    Vec3_t pos_w_;
    number_t fx_, fy_, cx_, cy_, focal_x_baseline_;
};

} // namespace se3
} // namespace g2o
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZER_G2O_SE3_PERSPECTIVE_POSE_OPT_EDGE_H
