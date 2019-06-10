#include "openvslam/data/frame.h"
#include "openvslam/initialize/bearing_vector.h"
#include "openvslam/solve/essential_solver.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace initialize {

bearing_vector::bearing_vector(const data::frame& ref_frm, const unsigned int max_num_iters)
        : base(ref_frm, max_num_iters) {
    spdlog::debug("CONSTRUCT: initialize::bearing_vector");
}

bearing_vector::~bearing_vector() {
    spdlog::debug("DESTRUCT: initialize::bearing_vector");
}

bool bearing_vector::initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) {
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

    // Eを計算
    auto essential_solver = solve::essential_solver(ref_bearings_, cur_bearings_, ref_cur_matches_);
    essential_solver.find_via_ransac(max_num_iters_);

    // 3次元点を作成
    if (essential_solver.solution_is_valid()) {
        const Mat33_t E_ref_to_cur = essential_solver.get_best_E_21();
        const auto is_inlier_match = essential_solver.get_inlier_matches();
        return reconstruct_with_E(E_ref_to_cur, is_inlier_match,
                                  rot_ref_to_cur_, trans_ref_to_cur_, triangulated_pts_, is_triangulated_);
    }
    else {
        return false;
    }
}

bool bearing_vector::reconstruct_with_E(const Mat33_t& E_ref_to_cur, const std::vector<bool>& is_inlier_match,
                                        Mat33_t& rot_ref_to_cur, Vec3_t& trans_ref_to_cur,
                                        eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                                        const float min_parallax_deg, const unsigned int min_num_triangulated) {
    // 4個の姿勢候補に対して，3次元点をtriangulationして有効な3次元点の数を数える

    // E行列を分解
    // https://en.wikipedia.org/wiki/Essential_matrix#Determining_R_and_t_from_E
    eigen_alloc_vector<Mat33_t> init_rots;
    eigen_alloc_vector<Vec3_t> init_transes;
    if (!solve::essential_solver::decompose(E_ref_to_cur, init_rots, init_transes)) {
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
                                          is_inlier_match, false,
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

    spdlog::info("initialization succeeded with E");
    return true;
}

} // namespace initialize
} // namespace openvslam
