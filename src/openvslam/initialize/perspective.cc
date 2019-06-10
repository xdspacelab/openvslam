#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/data/frame.h"
#include "openvslam/initialize/perspective.h"
#include "openvslam/solve/homography_solver.h"
#include "openvslam/solve/fundamental_solver.h"

#include <thread>

#include <spdlog/spdlog.h>

namespace openvslam {
namespace initialize {

perspective::perspective(const data::frame& ref_frm, const unsigned int max_num_iters)
        : base(ref_frm, max_num_iters), ref_cam_matrix_(get_camera_matrix(ref_frm.camera_)) {
    spdlog::debug("CONSTRUCT: initialize::perspective");
}

perspective::~perspective() {
    spdlog::debug("DESTRUCT: initialize::perspective");
}

bool perspective::initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) {
    // カメラモデルをセット
    cur_camera_ = cur_frm.camera_;
    // 特徴点を保存
    cur_undist_keypts_ = cur_frm.undist_keypts_;
    cur_bearings_ = cur_frm.bearings_;
    // matching情報を整形
    ref_cur_matches_.clear();
    ref_cur_matches_.reserve(cur_frm.undist_keypts_.size());
    for (unsigned int ref_idx = 0; ref_idx < ref_matches_with_cur.size(); ++ref_idx) {
        const auto cur_idx = ref_matches_with_cur.at(ref_idx);
        if (0 <= cur_idx) {
            ref_cur_matches_.emplace_back(std::make_pair(ref_idx, cur_idx));
        }
    }

    // set a camera matrix
    cur_cam_matrix_ = get_camera_matrix(cur_frm.camera_);

    // HとFを並列で計算
    auto homography_solver = solve::homography_solver(ref_undist_keypts_, cur_undist_keypts_, ref_cur_matches_, 1.0);
    auto fundamental_solver = solve::fundamental_solver(ref_undist_keypts_, cur_undist_keypts_, ref_cur_matches_, 1.0);
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

Mat33_t perspective::get_camera_matrix(camera::base* camera) {
    switch (camera->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective*>(camera);
            return c->eigen_cam_matrix_;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye*>(camera);
            return c->eigen_cam_matrix_;
        }
        default: {
            throw std::runtime_error("Cannot get a camera matrix from the camera model");
        }
    }
}

bool perspective::reconstruct_with_H(const Mat33_t& H_ref_to_cur, const std::vector<bool>& is_inlier_match,
                                     Mat33_t& rot_ref_to_cur, Vec3_t& trans_ref_to_cur,
                                     eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                                     const float min_parallax_deg, const unsigned int min_num_triangulated) {
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
        nums_valid_pts.at(i) = check_pose(init_rots.at(i), init_transes.at(i), 4.0,
                                          is_inlier_match, true,
                                          init_triangulated_pts.at(i), init_is_triangulated.at(i),
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
    if (*max_num_valid_pts_iter < min_num_triangulated) {
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
    // 4個の姿勢候補に対して，3次元点をtriangulationして有効な3次元点の数を数える

    // E行列を分解
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
        nums_valid_pts.at(i) = check_pose(init_rots.at(i), init_transes.at(i), 4.0,
                                          is_inlier_match, true,
                                          init_triangulated_pts.at(i), init_is_triangulated.at(i),
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
    if (*max_num_valid_pts_iter < min_num_triangulated) {
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

} // namespace initialize
} // namespace openvslam
