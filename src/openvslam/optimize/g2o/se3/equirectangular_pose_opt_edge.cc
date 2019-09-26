#include "openvslam/optimize/g2o/se3/equirectangular_pose_opt_edge.h"

namespace openvslam {
namespace optimize {
namespace g2o {
namespace se3 {

equirectangular_pose_opt_edge::equirectangular_pose_opt_edge()
    : ::g2o::BaseUnaryEdge<2, Vec2_t, shot_vertex>() {}

bool equirectangular_pose_opt_edge::read(std::istream& is) {
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

bool equirectangular_pose_opt_edge::write(std::ostream& os) const {
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

void equirectangular_pose_opt_edge::linearizeOplus() {
    auto vi = static_cast<shot_vertex*>(_vertices.at(0));
    const ::g2o::SE3Quat& cam_pose_cw = vi->shot_vertex::estimate();
    const Vec3_t pos_c = cam_pose_cw.map(pos_w_);

    const auto pcx = pos_c(0);
    const auto pcy = pos_c(1);
    const auto pcz = pos_c(2);
    const auto L = pos_c.norm();

    // 回転に対する微分
    const Vec3_t d_pc_d_rx(0, -pcz, pcy);
    const Vec3_t d_pc_d_ry(pcz, 0, -pcx);
    const Vec3_t d_pc_d_rz(-pcy, pcx, 0);
    // 並進に対する微分
    const Vec3_t d_pc_d_tx(1, 0, 0);
    const Vec3_t d_pc_d_ty(0, 1, 0);
    const Vec3_t d_pc_d_tz(0, 0, 1);

    // 状態ベクトルを x = [rx, ry, rz, tx, ty, tz] として，
    // 導関数ベクトル d_pcx_d_x, d_pcy_d_x, d_pcz_d_x を作成
    VecR_t<6> d_pcx_d_x;
    d_pcx_d_x << d_pc_d_rx(0), d_pc_d_ry(0), d_pc_d_rz(0),
        d_pc_d_tx(0), d_pc_d_ty(0), d_pc_d_tz(0);
    VecR_t<6> d_pcy_d_x;
    d_pcy_d_x << d_pc_d_rx(1), d_pc_d_ry(1), d_pc_d_rz(1),
        d_pc_d_tx(1), d_pc_d_ty(1), d_pc_d_tz(1);
    VecR_t<6> d_pcz_d_x;
    d_pcz_d_x << d_pc_d_rx(2), d_pc_d_ry(2), d_pc_d_rz(2),
        d_pc_d_tx(2), d_pc_d_ty(2), d_pc_d_tz(2);

    // 導関数ベクトル d_L_d_x を作成
    const Vec6_t d_L_d_x = (1.0 / L) * (pcx * d_pcx_d_x + pcy * d_pcy_d_x + pcz * d_pcz_d_x);

    // ヤコビ行列を作成
    MatRC_t<2, 6> jacobian = MatRC_t<2, 6>::Zero();
    jacobian.block<1, 6>(0, 0) = -(cols_ / (2 * M_PI)) * (1.0 / (pcx * pcx + pcz * pcz))
                                 * (pcz * d_pcx_d_x - pcx * d_pcz_d_x);
    jacobian.block<1, 6>(1, 0) = -(rows_ / M_PI) * (1.0 / (L * std::sqrt(pcx * pcx + pcz * pcz)))
                                 * (L * d_pcy_d_x - pcy * d_L_d_x);

    // g2oの変数にセット
    // 姿勢に対する微分
    _jacobianOplusXi = jacobian;
}

} // namespace se3
} // namespace g2o
} // namespace optimize
} // namespace openvslam
