#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/data/frame.h"
#include "openvslam/initialize/perspective.h"
#include "openvslam/solve/homography_solver.h"
#include "openvslam/solve/fundamental_solver.h"
#include "openvslam/solve/triangulator.h"

#include <thread>

#include <spdlog/spdlog.h>

namespace openvslam {
namespace initialize {

perspective::perspective(const data::frame& ref_frm, const float sigma, const unsigned int max_num_iters)
        : base(max_num_iters), sigma_(sigma), sigma_sq_(sigma * sigma) {
    spdlog::debug("CONSTRUCT: initialize::perspective");

    // カメラ行列をセット
    switch (ref_frm.camera_->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective*>(ref_frm.camera_);
            ref_cam_matrix_ = c->eigen_cam_matrix_;
            break;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye*>(ref_frm.camera_);
            ref_cam_matrix_ = c->eigen_cam_matrix_;
            break;
        }
        default: {
            throw std::runtime_error("Cannot initialize with equirectangular camera model");
        }
    }

    // 特徴点を保存
    ref_undist_keypts_ = ref_frm.undist_keypts_;
}

perspective::~perspective() {
    spdlog::debug("DESTRUCT: initialize::perspective");
}

bool perspective::initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) {
    // カメラ行列をセット
    switch (cur_frm.camera_->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective*>(cur_frm.camera_);
            cur_cam_matrix_ = c->eigen_cam_matrix_;
            break;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye*>(cur_frm.camera_);
            cur_cam_matrix_ = c->eigen_cam_matrix_;
            break;
        }
        default: {
            throw std::runtime_error("Cannot initialize with equirectangular camera model");
        }
    }

    // 特徴点を保存
    cur_undist_keypts_ = cur_frm.undist_keypts_;

    // matching情報を整形
    ref_cur_matches_.clear();
    ref_cur_matches_.reserve(cur_frm.undist_keypts_.size());
    for (unsigned int ref_idx = 0; ref_idx < ref_matches_with_cur.size(); ++ref_idx) {
        const auto cur_idx = ref_matches_with_cur.at(ref_idx);
        if (0 <= cur_idx) {
            ref_cur_matches_.emplace_back(std::make_pair(ref_idx, cur_idx));
        }
    }

    // HとFを並列で計算
    auto homography_solver = solve::homography_solver(ref_undist_keypts_, cur_undist_keypts_, ref_cur_matches_, sigma_);
    auto fundamental_solver = solve::fundamental_solver(ref_undist_keypts_, cur_undist_keypts_, ref_cur_matches_, sigma_);
    std::thread thread_for_H(&solve::homography_solver::find_via_ransac, &homography_solver, max_num_iters_, true);
    std::thread thread_for_F(&solve::fundamental_solver::find_via_ransac, &fundamental_solver, max_num_iters_, true);
    thread_for_H.join();
    thread_for_F.join();

    // スコア計算
    const auto score_H = homography_solver.get_best_score();
    const auto score_F = fundamental_solver.get_best_score();
    const float rel_score_H = score_H / (score_H + score_F);

    // ORB-SLAMの閾値に従って場合分け
    if (0.40 < rel_score_H && homography_solver.solution_is_valid()) {
        const Mat33_t H_ref_to_cur = homography_solver.get_best_H_21();
        const auto is_inlier_match = homography_solver.get_inlier_matches();
        return reconstruct_with_H(H_ref_to_cur, is_inlier_match,
                                  rot_ref_to_cur_, trans_ref_to_cur_, triangulated_pts_, is_triangulated_);
    }
    else if (fundamental_solver.solution_is_valid()) {
        const Mat33_t F_ref_to_cur = fundamental_solver.get_best_F_21();
        const auto is_inlier_match = fundamental_solver.get_inlier_matches();
        return reconstruct_with_F(F_ref_to_cur, is_inlier_match,
                                  rot_ref_to_cur_, trans_ref_to_cur_, triangulated_pts_, is_triangulated_);
    }
    else {
        return false;
    }
}

bool perspective::reconstruct_with_H(const Mat33_t& H_ref_to_cur, const std::vector<bool>& is_inlier_match,
                                     Mat33_t& rot_ref_to_cur, Vec3_t& trans_ref_to_cur,
                                     eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                                     const float min_parallax_deg, const unsigned int min_num_triangulated) {
    const auto num_inlier_matches = std::count(is_inlier_match.begin(), is_inlier_match.end(), true);

    // 8個の姿勢候補に対して，3次元点をtriangulationして有効な3次元点の数を数える

    // H行列を分解
    // Motion and structure from motion in a piecewise planar environment
    // (Faugeras et al. in IJPRAI 1988)
    eigen_alloc_vector<Mat33_t> init_rots;
    eigen_alloc_vector<Vec3_t> init_transes;
    eigen_alloc_vector<Vec3_t> init_normals;
    if (!solve::homography_solver::decompose(H_ref_to_cur, ref_cam_matrix_, cur_cam_matrix_, init_rots, init_transes, init_normals)) {
        return false;
    }

    constexpr unsigned int num_hypothesis = 8;

    // triangulationした3次元点座標
    std::array<eigen_alloc_vector<Vec3_t>, num_hypothesis> init_triangulated_pts;
    // 有効/無効フラグ
    std::array<std::vector<bool>, num_hypothesis> init_is_triangulated;
    // 各3次元点をtriangulationした際の視差
    std::array<float, num_hypothesis> init_parallax;
    // 有効な3次元点数
    std::array<unsigned int, num_hypothesis> nums_valid_pts;

    for (unsigned int i = 0; i < num_hypothesis; ++i) {
        nums_valid_pts.at(i) = check_pose(init_rots.at(i), init_transes.at(i), 4.0 * sigma_sq_,
                                          is_inlier_match, init_triangulated_pts.at(i), init_is_triangulated.at(i),
                                          init_parallax.at(i));
    }

    rot_ref_to_cur = Mat33_t::Zero();
    trans_ref_to_cur = Vec3_t::Zero();

    // triangulationされた3次元点数の最大値を求める
    // nums_valid_ptsのイテレータ
    const auto max_num_valid_pts_iter = std::max_element(nums_valid_pts.begin(), nums_valid_pts.end());
    // index
    const auto max_num_valid_index = std::distance(nums_valid_pts.begin(), max_num_valid_pts_iter);

    // 3次元点数の条件を満たしていなければ破棄
    const auto min_num_valid_pts = std::max(static_cast<unsigned int>(0.9 * num_inlier_matches), min_num_triangulated);
    if (*max_num_valid_pts_iter < min_num_valid_pts) {
        return false;
    }

    // どれが正しい姿勢かはっきりしなければ破棄
    const auto num_similars = std::count_if(nums_valid_pts.begin(), nums_valid_pts.end(),
                                            [max_num_valid_pts_iter](unsigned int num_valid_pts) {
                                                return 0.7 * (*max_num_valid_pts_iter) < num_valid_pts;
                                            });

    if (1 < num_similars) {
        return false;
    }

    // 視差が小さければ破棄
    if (init_parallax.at(max_num_valid_index) < min_parallax_deg) {
        return false;
    }

    // reconstructionに成功したので情報を保存
    triangulated_pts = init_triangulated_pts.at(max_num_valid_index);
    is_triangulated = init_is_triangulated.at(max_num_valid_index);
    rot_ref_to_cur = init_rots.at(max_num_valid_index);
    trans_ref_to_cur = init_transes.at(max_num_valid_index);

    spdlog::info("initialization succeeded with H");
    return true;
}

bool perspective::reconstruct_with_F(const Mat33_t& F_ref_to_cur, const std::vector<bool>& is_inlier_match,
                                     Mat33_t& rot_ref_to_cur, Vec3_t& trans_ref_to_cur,
                                     eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                                     const float min_parallax_deg, const unsigned int min_num_triangulated) {
    const auto num_inlier_matches = std::count(is_inlier_match.begin(), is_inlier_match.end(), true);

    // 4個の姿勢候補に対して，3次元点をtriangulationして有効な3次元点の数を数える

    // E行列を分解
    // https://en.wikipedia.org/wiki/Essential_matrix#Determining_R_and_t_from_E
    eigen_alloc_vector<Mat33_t> init_rots;
    eigen_alloc_vector<Vec3_t> init_transes;
    if (!solve::fundamental_solver::decompose(F_ref_to_cur, ref_cam_matrix_, cur_cam_matrix_, init_rots, init_transes)) {
        return false;
    }

    constexpr unsigned int num_hypothesis = 4;

    // triangulationした3次元点座標
    std::array<eigen_alloc_vector<Vec3_t>, num_hypothesis> init_triangulated_pts;
    // 有効/無効フラグ
    std::array<std::vector<bool>, num_hypothesis> init_is_triangulated;
    // 各3次元点をtriangulationした際の視差
    std::array<float, num_hypothesis> init_parallax;
    // 有効な3次元点数
    std::array<unsigned int, num_hypothesis> nums_valid_pts;

    for (unsigned int i = 0; i < num_hypothesis; ++i) {
        nums_valid_pts.at(i) = check_pose(init_rots.at(i), init_transes.at(i), 4.0 * sigma_sq_,
                                          is_inlier_match, init_triangulated_pts.at(i), init_is_triangulated.at(i),
                                          init_parallax.at(i));
    }

    rot_ref_to_cur = Mat33_t::Zero();
    trans_ref_to_cur = Vec3_t::Zero();

    // triangulationされた3次元点数の最大値を求める
    // nums_valid_ptsのイテレータ
    const auto max_num_valid_pts_iter = std::max_element(nums_valid_pts.begin(), nums_valid_pts.end());
    // index
    const auto max_num_valid_index = std::distance(nums_valid_pts.begin(), max_num_valid_pts_iter);

    // 3次元点数の条件を満たしていなければ破棄
    const auto min_num_valid_pts = std::max(static_cast<unsigned int>(0.9 * num_inlier_matches), min_num_triangulated);
    if (*max_num_valid_pts_iter < min_num_valid_pts) {
        return false;
    }

    // どれが正しい姿勢かはっきりしなければ破棄
    const auto num_similars = std::count_if(nums_valid_pts.begin(), nums_valid_pts.end(),
                                            [max_num_valid_pts_iter](unsigned int num_valid_pts) {
                                                return 0.7 * (*max_num_valid_pts_iter) < num_valid_pts;
                                            });

    if (1 < num_similars) {
        return false;
    }

    // 視差が小さければ破棄
    if (init_parallax.at(max_num_valid_index) < min_parallax_deg) {
        return false;
    }

    // reconstructionに成功したので情報を保存
    triangulated_pts = init_triangulated_pts.at(max_num_valid_index);
    is_triangulated = init_is_triangulated.at(max_num_valid_index);
    rot_ref_to_cur = init_rots.at(max_num_valid_index);
    trans_ref_to_cur = init_transes.at(max_num_valid_index);

    spdlog::info("initialization succeeded with F");
    return true;
}

unsigned int perspective::check_pose(const Mat33_t& rot_ref_to_cur, const Vec3_t& trans_ref_to_cur,
                                     const float reproj_err_thr_sq, const std::vector<bool>& is_inlier_match,
                                     eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                                     float& parallax) {
    // = cos(0.5deg)
    constexpr float cos_parallax_thr = 0.99996192306;

    is_triangulated.resize(ref_undist_keypts_.size(), false);
    triangulated_pts.resize(ref_undist_keypts_.size());

    std::vector<float> cos_parallaxes;
    cos_parallaxes.reserve(ref_undist_keypts_.size());

    // referenceの透視投影行列(原点として設定する)
    Mat34_t P_ref = Mat34_t::Zero();
    P_ref.block<3, 3>(0, 0) = ref_cam_matrix_;
    // カメラ中心
    const Vec3_t ref_cam_center = Vec3_t::Zero();

    // currentの透視投影行列(reference基準のの相対姿勢として設定する)
    Mat34_t P_cur = Mat34_t::Zero();
    P_cur.block<3, 3>(0, 0) = rot_ref_to_cur;
    P_cur.block<3, 1>(0, 3) = trans_ref_to_cur;
    P_cur = cur_cam_matrix_ * P_cur;
    // カメラ中心
    const Vec3_t cur_cam_center = -rot_ref_to_cur.transpose() * trans_ref_to_cur;

    unsigned int num_valid_pts = 0;

    for (unsigned int i = 0; i < ref_cur_matches_.size(); ++i) {
        if (!is_inlier_match.at(i)) {
            continue;
        }

        const auto& ref_undist_keypt = ref_undist_keypts_.at(ref_cur_matches_.at(i).first);
        const auto& cur_undist_keypt = cur_undist_keypts_.at(ref_cur_matches_.at(i).second);

        const Vec3_t pos_c_in_ref = solve::triangulator::triangulate(ref_undist_keypt.pt, cur_undist_keypt.pt, P_ref, P_cur);

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
        // 3次元点がreferenceのカメラの前にあることを確認
        if (cos_parallax < cos_parallax_thr && pos_c_in_ref(2) <= 0) {
            continue;
        }

        // 視野角が十分あることを確認
        // 3次元点がcurrentのカメラの前にあることを確認
        const Vec3_t pos_c_in_cur = rot_ref_to_cur * pos_c_in_ref + trans_ref_to_cur;
        if (cos_parallax < cos_parallax_thr && pos_c_in_cur(2) <= 0) {
            continue;
        }

        // referenceの画像で再投影誤差を計算
        const float ref_reproj_x = ref_cam_matrix_(0, 0) * pos_c_in_ref(0) / pos_c_in_ref(2) + ref_cam_matrix_(0, 2);
        const float ref_reproj_y = ref_cam_matrix_(1, 1) * pos_c_in_ref(1) / pos_c_in_ref(2) + ref_cam_matrix_(1, 2);

        const float ref_reproj_err_sq = (ref_reproj_x - ref_undist_keypt.pt.x) * (ref_reproj_x - ref_undist_keypt.pt.x)
                                        + (ref_reproj_y - ref_undist_keypt.pt.y) * (ref_reproj_y - ref_undist_keypt.pt.y);

        if (reproj_err_thr_sq < ref_reproj_err_sq) {
            continue;
        }

        // currentの画像で再投影誤差を計算
        const float cur_reproj_x = cur_cam_matrix_(0, 0) * pos_c_in_cur(0) / pos_c_in_cur(2) + cur_cam_matrix_(0, 2);
        const float cur_reproj_y = cur_cam_matrix_(1, 1) * pos_c_in_cur(1) / pos_c_in_cur(2) + cur_cam_matrix_(1, 2);

        const float cur_reproj_err_sq = (cur_reproj_x - cur_undist_keypt.pt.x) * (cur_reproj_x - cur_undist_keypt.pt.x)
                                        + (cur_reproj_y - cur_undist_keypt.pt.y) * (cur_reproj_y - cur_undist_keypt.pt.y);

        if (reproj_err_thr_sq < cur_reproj_err_sq) {
            continue;
        }

        cos_parallaxes.push_back(cos_parallax);
        triangulated_pts.at(ref_cur_matches_.at(i).first) = pos_c_in_ref;
        ++num_valid_pts;
        // 視野角が十分あればreconstructionに用いる
        if (cos_parallax < cos_parallax_thr) {
            is_triangulated.at(ref_cur_matches_.at(i).first) = true;
        }
    }

    if (0 < num_valid_pts) {
        // 50番目に大きい視差角で評価
        std::sort(cos_parallaxes.begin(), cos_parallaxes.end());
        const auto idx = std::min(50, static_cast<int>(cos_parallaxes.size() - 1));
        parallax = std::acos(cos_parallaxes.at(idx)) * 180.0 / M_PI;
    }
    else {
        parallax = 0.0;
    }

    return num_valid_pts;
}

} // namespace initialize
} // namespace openvslam
