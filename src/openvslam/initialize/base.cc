#include "openvslam/data/frame.h"
#include "openvslam/initialize/base.h"
#include "openvslam/solve/triangulator.h"

namespace openvslam {
namespace initialize {

base::base(const data::frame& ref_frm, const unsigned int max_num_iters)
        : ref_camera_(ref_frm.camera_), ref_undist_keypts_(ref_frm.undist_keypts_), ref_bearings_(ref_frm.bearings_),
          max_num_iters_(max_num_iters) {}

Mat33_t base::get_rotation_ref_to_cur() const {
    return rot_ref_to_cur_;
}

Vec3_t base::get_translation_ref_to_cur() const {
    return trans_ref_to_cur_;
}

eigen_alloc_vector<Vec3_t> base::get_triangulated_pts() const {
    return triangulated_pts_;
}

std::vector<bool> base::get_triangulated_flags() const {
    return is_triangulated_;
}

unsigned int base::check_pose(const Mat33_t& rot_ref_to_cur, const Vec3_t& trans_ref_to_cur, const float reproj_err_thr_sq,
                              const std::vector<bool>& is_inlier_match, const bool depth_is_positive,
                              eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                              float& parallax_deg) {
    // = cos(0.5deg)
    constexpr float cos_parallax_thr = 0.99996192306;

    is_triangulated.resize(ref_undist_keypts_.size(), false);
    triangulated_pts.resize(ref_undist_keypts_.size());

    std::vector<float> cos_parallaxes;
    cos_parallaxes.reserve(ref_undist_keypts_.size());

    // referenceのカメラ中心
    const Vec3_t ref_cam_center = Vec3_t::Zero();

    // currentのカメラ中心
    const Vec3_t cur_cam_center = -rot_ref_to_cur.transpose() * trans_ref_to_cur;

    unsigned int num_valid_pts = 0;

    for (unsigned int i = 0; i < ref_cur_matches_.size(); ++i) {
        if (!is_inlier_match.at(i)) {
            continue;
        }

        const Vec3_t& ref_bearing = ref_bearings_.at(ref_cur_matches_.at(i).first);
        const Vec3_t& cur_bearing = cur_bearings_.at(ref_cur_matches_.at(i).second);

        const Vec3_t pos_c_in_ref = solve::triangulator::triangulate(ref_bearing, cur_bearing, rot_ref_to_cur, trans_ref_to_cur);

        if (!std::isfinite(pos_c_in_ref(0))
            || !std::isfinite(pos_c_in_ref(1))
            || !std::isfinite(pos_c_in_ref(2))) {
            continue;
        }

        // 視差角を計算
        const Vec3_t ref_normal = pos_c_in_ref - ref_cam_center;
        const float ref_norm = ref_normal.norm();
        const Vec3_t cur_normal = pos_c_in_ref - cur_cam_center;
        const float cur_norm = cur_normal.norm();
        const float cos_parallax = ref_normal.dot(cur_normal) / (ref_norm * cur_norm);

        // 視野角が十分あることを確認
        if (cos_parallax_thr < cos_parallax) {
            continue;
        }

        // 3次元点がカメラの前にあることを確認
        if (depth_is_positive) {
            if (pos_c_in_ref(2) <= 0) {
                continue;
            }
            const Vec3_t pos_c_in_cur = rot_ref_to_cur * pos_c_in_ref + trans_ref_to_cur;
            if (pos_c_in_cur(2) <= 0) {
                continue;
            }
        }

        const auto& ref_undist_keypt = ref_undist_keypts_.at(ref_cur_matches_.at(i).first);
        const auto& cur_undist_keypt = cur_undist_keypts_.at(ref_cur_matches_.at(i).second);

        // referenceの画像で再投影誤差を計算
        Vec2_t reproj_in_ref;
        float x_right_in_ref;
        const auto is_valid_ref = ref_camera_->reproject_to_image(Mat33_t::Identity(), Vec3_t::Zero(), pos_c_in_ref,
                                                                  reproj_in_ref, x_right_in_ref);
        if (!is_valid_ref) {
            continue;
        }

        const float ref_reproj_err_sq = (reproj_in_ref - ref_undist_keypt.pt).squaredNorm();
        if (reproj_err_thr_sq < ref_reproj_err_sq) {
            continue;
        }

        // currentの画像で再投影誤差を計算
        Vec2_t reproj_in_cur;
        float x_right_in_cur;
        const auto is_valid_cur = cur_camera_->reproject_to_image(rot_ref_to_cur, trans_ref_to_cur, pos_c_in_ref,
                                                                  reproj_in_cur, x_right_in_cur);
        if (!is_valid_cur) {
            continue;
        }
        const float cur_reproj_err_sq = (reproj_in_cur - cur_undist_keypt.pt).squaredNorm();
        if (reproj_err_thr_sq < cur_reproj_err_sq) {
            continue;
        }

        triangulated_pts.at(ref_cur_matches_.at(i).first) = pos_c_in_ref;
        is_triangulated.at(ref_cur_matches_.at(i).first) = true;
        ++num_valid_pts;

        cos_parallaxes.push_back(cos_parallax);
    }

    if (0 < num_valid_pts) {
        // 50番目に大きい視差角で評価
        std::sort(cos_parallaxes.begin(), cos_parallaxes.end());
        const auto idx = std::min(50, static_cast<int>(cos_parallaxes.size() - 1));
        parallax_deg = std::acos(cos_parallaxes.at(idx)) * 180.0 / M_PI;
    }
    else {
        parallax_deg = 0.0;
    }

    return num_valid_pts;
}

} // namespace initialize
} // namespace openvslam
