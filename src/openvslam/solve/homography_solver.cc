#include "openvslam/solve/common.h"
#include "openvslam/solve/homography_solver.h"
#include "openvslam/util/converter.h"
#include "openvslam/util/random_array.h"

namespace openvslam {
namespace solve {

homography_solver::homography_solver(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
                                     const std::vector<std::pair<int, int>>& matches_12, const float sigma)
        : undist_keypts_1_(undist_keypts_1), undist_keypts_2_(undist_keypts_2), matches_12_(matches_12), sigma_(sigma) {}

void homography_solver::find_via_ransac(const unsigned int max_num_iter, const bool recompute) {
    const auto num_matches = static_cast<unsigned int>(matches_12_.size());

    // 1. 特徴点座標を正規化する

    // 正規化した特徴点座標
    std::vector<cv::Point2f> normalized_keypts_1, normalized_keypts_2;
    // 正規化する際のtransform
    Mat33_t transform_1, transform_2;
    // 正規化する
    normalize(undist_keypts_1_, normalized_keypts_1, transform_1);
    normalize(undist_keypts_2_, normalized_keypts_2, transform_2);

    const Mat33_t transform_2_inv = transform_2.inverse();

    // 2. RANSACを回す

    // 2-1. 変数の準備

    // 最小サンプル数
    constexpr unsigned int min_set_size = 8;
    if (num_matches < min_set_size) {
        solution_is_valid_ = false;
        return;
    }
    // RANSAC用変数
    best_score_ = 0.0;
    is_inlier_match_ = std::vector<bool>(num_matches, false);
    // 最小セットの特徴点
    std::vector<cv::Point2f> min_set_keypts_1(min_set_size);
    std::vector<cv::Point2f> min_set_keypts_2(min_set_size);
    // RANSACループ内で使い回す変数
    // H_21_in_sac: 1->2のhomography, H_12_in_sac: 2->1のhomography
    Mat33_t H_21_in_sac, H_12_in_sac;
    // inlier/outlierのフラグ
    std::vector<bool> is_inlier_match_in_sac(num_matches, false);
    // スコア
    float score_in_sac;

    // RANSACを回してH行列を求める
    for (unsigned int iter = 0; iter < max_num_iter; ++iter) {
        // 2-2. RANSACセットを作る
        const auto indices = util::create_random_array(min_set_size, 0U, num_matches - 1);
        for (unsigned int i = 0; i < min_set_size; ++i) {
            const auto idx = indices.at(i);
            min_set_keypts_1.at(i) = normalized_keypts_1.at(matches_12_.at(idx).first);
            min_set_keypts_2.at(i) = normalized_keypts_2.at(matches_12_.at(idx).second);
        }

        // 2-3. H行列を求める
        const Mat33_t normalized_H_21 = compute_H_21(min_set_keypts_1, min_set_keypts_2);

        // 2-4. 正規化前の画像座標に戻す
        H_21_in_sac = transform_2_inv * normalized_H_21 * transform_1;

        // 2-5. インライアチェック
        score_in_sac = check_inliers(H_21_in_sac, is_inlier_match_in_sac);

        // 2-6. ベストモデルの更新
        if (best_score_ < score_in_sac) {
            best_score_ = score_in_sac;
            best_H_21_ = H_21_in_sac;
            is_inlier_match_ = is_inlier_match_in_sac;
        }
    }

    if (0.0 < best_score_) {
        solution_is_valid_ = true;
    }

    if (!recompute || !solution_is_valid_) {
        return;
    }

    // インライアのみでもう一度H行列を求める
    std::vector<cv::Point2f> inlier_normalized_keypts_1;
    std::vector<cv::Point2f> inlier_normalized_keypts_2;
    inlier_normalized_keypts_1.reserve(matches_12_.size());
    inlier_normalized_keypts_2.reserve(matches_12_.size());
    for (unsigned int i = 0; i < matches_12_.size(); ++i) {
        if (is_inlier_match_.at(i)) {
            inlier_normalized_keypts_1.push_back(normalized_keypts_1.at(matches_12_.at(i).first));
            inlier_normalized_keypts_2.push_back(normalized_keypts_2.at(matches_12_.at(i).second));
        }
    }

    // スコアをもう一度計算する
    const Mat33_t normalized_H_21 = solve::homography_solver::compute_H_21(inlier_normalized_keypts_1, inlier_normalized_keypts_2);
    best_H_21_ = transform_2_inv * normalized_H_21 * transform_1;
    best_score_ = check_inliers(best_H_21_, is_inlier_match_);
}

float homography_solver::check_inliers(const Mat33_t& H_21, std::vector<bool>& is_inlier_match) {
    const auto num_matches = matches_12_.size();

    // 自由度2のカイ2乗分布におけるカイ2乗値(上端有意確率0.05)
    // (homography transformの誤差は2自由度)
    constexpr float chi_sq_thr = 5.991;

    is_inlier_match.resize(num_matches);

    const Mat33_t H_12 = H_21.inverse();

    float score = 0;

    const float inv_sigma_sq = 1.0 / (sigma_ * sigma_);

    for (unsigned int i = 0; i < num_matches; ++i) {
        const auto& keypt_1 = undist_keypts_1_.at(matches_12_.at(i).first);
        const auto& keypt_2 = undist_keypts_2_.at(matches_12_.at(i).second);

        // 1. homography変換するため同次座標に変換

        const Vec3_t pt_1 = util::converter::to_homogeneous(keypt_1.pt);
        const Vec3_t pt_2 = util::converter::to_homogeneous(keypt_2.pt);

        // 2. symmetric transfer errorを計算

        // 2-1. 1の点を2の画像上に変換し，transfer error(点と点の距離)を計算

        Vec3_t transformed_pt_1 = H_21 * pt_1;
        transformed_pt_1 = transformed_pt_1 / transformed_pt_1(2);

        const float dist_sq_1 = (pt_2 - transformed_pt_1).squaredNorm();

        // 分散で標準化する
        const float chi_sq_1 = dist_sq_1 * inv_sigma_sq;

        // インライアであればスコアを計算する
        if (chi_sq_thr < chi_sq_1) {
            is_inlier_match.at(i) = false;
            continue;
        }
        else {
            is_inlier_match.at(i) = true;
            score += chi_sq_thr - chi_sq_1;
        }

        // 2-2. 2の点を1の画像上に変換し，transfer error(点と点の距離)を計算

        Vec3_t transformed_pt_2 = H_12 * pt_2;
        transformed_pt_2 = transformed_pt_2 / transformed_pt_2(2);

        const float dist_sq_2 = (pt_1 - transformed_pt_2).squaredNorm();

        // 分散で標準化する
        const float chi_sq_2 = dist_sq_2 * inv_sigma_sq;

        // インライアであればスコアを計算する
        if (chi_sq_thr < chi_sq_2) {
            is_inlier_match.at(i) = false;
            continue;
        }
        else {
            is_inlier_match.at(i) = true;
            score += chi_sq_thr - chi_sq_2;
        }
    }

    return score;
}

Mat33_t homography_solver::compute_H_21(const std::vector<cv::Point2f>& keypts_1, const std::vector<cv::Point2f>& keypts_2) {
    // https://www.uio.no/studier/emner/matnat/its/UNIK4690/v16/forelesninger/lecture_4_3-estimating-homographies-from-feature-correspondences.pdf

    assert(keypts_1.size() == keypts_2.size());

    const auto num_points = keypts_1.size();

    typedef Eigen::Matrix<Mat33_t::Scalar, Eigen::Dynamic, 9> CoeffMatrix;
    CoeffMatrix A(2 * num_points, 9);

    for (unsigned int i = 0; i < num_points; ++i) {
        const float x_1 = keypts_1.at(i).x;
        const float y_1 = keypts_1.at(i).y;
        const float x_2 = keypts_2.at(i).x;
        const float y_2 = keypts_2.at(i).y;

        A(2 * i, 0) = 0.0;
        A(2 * i, 1) = 0.0;
        A(2 * i, 2) = 0.0;
        A(2 * i, 3) = -x_1;
        A(2 * i, 4) = -y_1;
        A(2 * i, 5) = -1;
        A(2 * i, 6) = y_2 * x_1;
        A(2 * i, 7) = y_2 * y_1;
        A(2 * i, 8) = y_2;

        A(2 * i + 1, 0) = x_1;
        A(2 * i + 1, 1) = y_1;
        A(2 * i + 1, 2) = 1;
        A(2 * i + 1, 3) = 0.0;
        A(2 * i + 1, 4) = 0.0;
        A(2 * i + 1, 5) = 0.0;
        A(2 * i + 1, 6) = -x_2 * x_1;
        A(2 * i + 1, 7) = -x_2 * y_1;
        A(2 * i + 1, 8) = -x_2;
    }

    const Eigen::JacobiSVD<CoeffMatrix> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix<Mat33_t::Scalar, 9, 1> v = svd.matrixV().col(8);
    // pointerをコンストラクタに渡した場合，col-majorで値が格納されるため，transposeする必要がある
    const Mat33_t H_21 = Mat33_t(v.data()).transpose();

    return H_21;
}

bool homography_solver::decompose(const Mat33_t& H_21, const Mat33_t& cam_matrix_1, const Mat33_t& cam_matrix_2,
                                  eigen_alloc_vector<Mat33_t>& init_rots, eigen_alloc_vector<Vec3_t>& init_transes, eigen_alloc_vector<Vec3_t>& init_normals) {
    // Motion and structure from motion in a piecewise planar environment
    // (Faugeras et al. in IJPRAI 1988)

    init_rots.reserve(8);
    init_transes.reserve(8);
    init_normals.reserve(8);

    const Mat33_t A = cam_matrix_2.inverse() * H_21 * cam_matrix_1;

    // 式(7)のSVD
    const Eigen::JacobiSVD<Mat33_t> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Mat33_t& U = svd.matrixU();
    const Vec3_t& lambda = svd.singularValues();
    const Mat33_t& V = svd.matrixV();
    const Mat33_t Vt = V.transpose();

    const float d1 = lambda(0);
    const float d2 = lambda(1);
    const float d3 = lambda(2);

    // ランク条件を確認
    if (d1 / d2 < 1.0001 || d2 / d3 < 1.0001) {
        return false;
    }

    // 式(8)の中間変数
    const float s = U.determinant() * Vt.determinant();

    // 式(12)のx1とx3
    const float aux_1 = std::sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    const float aux_3 = std::sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
    const std::array<float, 4> x1s = {{aux_1, aux_1, -aux_1, -aux_1}};
    const std::array<float, 4> x3s = {{aux_3, -aux_3, aux_3, -aux_3}};

    // d'が正の場合

    // 式(13)
    const float aux_sin_theta = std::sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);
    const float cos_theta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
    const std::array<float, 4> aux_sin_thetas = {{aux_sin_theta, -aux_sin_theta, -aux_sin_theta, aux_sin_theta}};

    for (unsigned int i = 0; i < 4; ++i) {
        // 式(13)
        Mat33_t aux_rot = Mat33_t::Identity();
        aux_rot(0, 0) = cos_theta;
        aux_rot(0, 2) = -aux_sin_thetas.at(i);
        aux_rot(2, 0) = aux_sin_thetas.at(i);
        aux_rot(2, 2) = cos_theta;
        // 式(8)
        const Mat33_t init_rot = s * U * aux_rot * Vt;
        init_rots.push_back(init_rot);

        // 式(14)
        Vec3_t aux_trans{x1s.at(i), 0.0, -x3s.at(i)};
        aux_trans *= d1 - d3;
        // 式(8)
        const Vec3_t init_trans = U * aux_trans;
        init_transes.emplace_back(init_trans / init_trans.norm());

        // 式(9)
        const Vec3_t aux_normal{x1s.at(i), 0.0, x3s.at(i)};
        // 式(8)
        Vec3_t init_normal = V * aux_normal;
        if (init_normal(2) < 0) {
            init_normal = -init_normal;
        }
        init_normals.push_back(init_normal);
    }

    // d'が負の場合
    const float aux_sin_phi = std::sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);

    const float cos_phi = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
    const std::array<float, 4> sin_phis = {{aux_sin_phi, -aux_sin_phi, -aux_sin_phi, aux_sin_phi}};

    for (unsigned int i = 0; i < 4; ++i) {
        // 式(15)
        Mat33_t aux_rot = Mat33_t::Identity();
        aux_rot(0, 0) = cos_phi;
        aux_rot(0, 2) = sin_phis.at(i);
        aux_rot(1, 1) = -1;
        aux_rot(2, 0) = sin_phis.at(i);
        aux_rot(2, 2) = -cos_phi;
        // 式(8)
        const Mat33_t init_rot = s * U * aux_rot * Vt;
        init_rots.push_back(init_rot);

        // 式(16)
        Vec3_t aux_trans{x1s.at(i), 0.0, x3s.at(i)};
        aux_trans(0) = x1s.at(i);
        aux_trans(1) = 0;
        aux_trans(2) = x3s.at(i);
        aux_trans *= d1 + d3;
        // 式(8)
        const Vec3_t init_trans = U * aux_trans;
        init_transes.emplace_back(init_trans / init_trans.norm());

        // 式(9)
        const Vec3_t aux_normal{x1s.at(i), 0.0, x3s.at(i)};
        Vec3_t init_normal = V * aux_normal;
        if (init_normal(2) < 0) {
            init_normal = -init_normal;
        }
        init_normals.push_back(init_normal);
    }

    return true;
}

} // namespace solver
} // namespace openvslam
