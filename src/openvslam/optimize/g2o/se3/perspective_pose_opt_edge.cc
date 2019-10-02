#include "openvslam/optimize/g2o/se3/perspective_pose_opt_edge.h"

namespace openvslam {
namespace optimize {
namespace g2o {
namespace se3 {

mono_perspective_pose_opt_edge::mono_perspective_pose_opt_edge()
    : ::g2o::BaseUnaryEdge<2, Vec2_t, shot_vertex>() {}

bool mono_perspective_pose_opt_edge::read(std::istream& is) {
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

bool mono_perspective_pose_opt_edge::write(std::ostream& os) const {
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

void mono_perspective_pose_opt_edge::linearizeOplus() {
    auto vi = static_cast<shot_vertex*>(_vertices.at(0));
    const ::g2o::SE3Quat& cam_pose_cw = vi->shot_vertex::estimate();
    const Vec3_t pos_c = cam_pose_cw.map(pos_w_);

    const auto x = pos_c(0);
    const auto y = pos_c(1);
    const auto z = pos_c(2);
    const auto z_sq = z * z;

    _jacobianOplusXi(0, 0) = x * y / z_sq * fx_;
    _jacobianOplusXi(0, 1) = -(1.0 + (x * x / z_sq)) * fx_;
    _jacobianOplusXi(0, 2) = y / z * fx_;
    _jacobianOplusXi(0, 3) = -1.0 / z * fx_;
    _jacobianOplusXi(0, 4) = 0;
    _jacobianOplusXi(0, 5) = x / z_sq * fx_;

    _jacobianOplusXi(1, 0) = (1.0 + y * y / z_sq) * fy_;
    _jacobianOplusXi(1, 1) = -x * y / z_sq * fy_;
    _jacobianOplusXi(1, 2) = -x / z * fy_;
    _jacobianOplusXi(1, 3) = 0.0;
    _jacobianOplusXi(1, 4) = -1.0 / z * fy_;
    _jacobianOplusXi(1, 5) = y / z_sq * fy_;
}

stereo_perspective_pose_opt_edge::stereo_perspective_pose_opt_edge()
    : ::g2o::BaseUnaryEdge<3, Vec3_t, shot_vertex>() {}

bool stereo_perspective_pose_opt_edge::read(std::istream& is) {
    for (unsigned int i = 0; i < 3; ++i) {
        is >> _measurement(i);
    }
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = i; j < 3; ++j) {
            is >> information()(i, j);
            if (i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }
    return true;
}

bool stereo_perspective_pose_opt_edge::write(std::ostream& os) const {
    for (unsigned int i = 0; i < 3; ++i) {
        os << measurement()(i) << " ";
    }
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = i; j < 3; ++j) {
            os << " " << information()(i, j);
        }
    }
    return os.good();
}

void stereo_perspective_pose_opt_edge::linearizeOplus() {
    auto vi = static_cast<shot_vertex*>(_vertices.at(0));
    const ::g2o::SE3Quat& cam_pose_cw = vi->shot_vertex::estimate();
    const Vec3_t pos_c = cam_pose_cw.map(pos_w_);

    const auto x = pos_c(0);
    const auto y = pos_c(1);
    const auto z = pos_c(2);
    const auto z_sq = z * z;

    _jacobianOplusXi(0, 0) = x * y / z_sq * fx_;
    _jacobianOplusXi(0, 1) = -(1.0 + (x * x / z_sq)) * fx_;
    _jacobianOplusXi(0, 2) = y / z * fx_;
    _jacobianOplusXi(0, 3) = -1.0 / z * fx_;
    _jacobianOplusXi(0, 4) = 0.0;
    _jacobianOplusXi(0, 5) = x / z_sq * fx_;

    _jacobianOplusXi(1, 0) = (1.0 + y * y / z_sq) * fy_;
    _jacobianOplusXi(1, 1) = -x * y / z_sq * fy_;
    _jacobianOplusXi(1, 2) = -x / z * fy_;
    _jacobianOplusXi(1, 3) = 0.0;
    _jacobianOplusXi(1, 4) = -1.0 / z * fy_;
    _jacobianOplusXi(1, 5) = y / z_sq * fy_;

    _jacobianOplusXi(2, 0) = _jacobianOplusXi(0, 0) - focal_x_baseline_ * y / z_sq;
    _jacobianOplusXi(2, 1) = _jacobianOplusXi(0, 1) + focal_x_baseline_ * x / z_sq;
    _jacobianOplusXi(2, 2) = _jacobianOplusXi(0, 2);
    _jacobianOplusXi(2, 3) = _jacobianOplusXi(0, 3);
    _jacobianOplusXi(2, 4) = 0.0;
    _jacobianOplusXi(2, 5) = _jacobianOplusXi(0, 5) - focal_x_baseline_ / z_sq;
}

} // namespace se3
} // namespace g2o
} // namespace optimize
} // namespace openvslam
