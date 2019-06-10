#include "openvslam/solve/common.h"
#include "openvslam/solve/fundamental_solver.h"
#include "openvslam/util/converter.h"
#include "openvslam/util/random_array.h"

namespace openvslam {
namespace solve {

fundamental_solver::fundamental_solver(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
                                       const std::vector<std::pair<int, int>>& matches_12, const float sigma)
        : undist_keypts_1_(undist_keypts_1), undist_keypts_2_(undist_keypts_2), matches_12_(matches_12), sigma_(sigma) {}

void fundamental_solver::find_via_ransac(const unsigned int max_num_iter, const bool recompute) {
    const auto num_matches = static_cast<unsigned int>(matches_12_.size());

    // 1. 特徴点座標を正規化する

    // 正規化した特徴点座標
    std::vector<cv::Point2f> normalized_keypts_1, normalized_keypts_2;
    // 正規化する際のtransform
    Mat33_t transform_1, transform_2;
    // 正規化する
    normalize(undist_keypts_1_, normalized_keypts_1, transform_1);
    normalize(undist_keypts_2_, normalized_keypts_2, transform_2);

    const Mat33_t transform_2_t = transform_2.transpose();

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
    // F_21_in_sac: 1->2のfundamental
    Mat33_t F_21_in_sac;
    // inlier/outlierのフラグ
    std::vector<bool> is_inlier_match_in_sac(num_matches, false);
    // スコア
    float score_in_sac;

    // RANSACを回してF行列を求める
    for (unsigned int iter = 0; iter < max_num_iter; iter++) {
        // 2-2. RANSACセットを作る
        const auto indices = util::create_random_array(min_set_size, 0U, num_matches - 1);
        for (unsigned int i = 0; i < min_set_size; ++i) {
            const auto idx = indices.at(i);
            min_set_keypts_1.at(i) = normalized_keypts_1.at(matches_12_.at(idx).first);
            min_set_keypts_2.at(i) = normalized_keypts_2.at(matches_12_.at(idx).second);
        }

        // 2-3. F行列を求める
        const Mat33_t normalized_F_21 = compute_F_21(min_set_keypts_1, min_set_keypts_2);

        // 2-4. 正規化前の画像座標に戻す
        F_21_in_sac = transform_2_t * normalized_F_21 * transform_1;

        // 2-5. インライアチェック
        score_in_sac = check_inliers(F_21_in_sac, is_inlier_match_in_sac);

        // 2-6. ベストモデルの更新
        if (best_score_ < score_in_sac) {
            best_score_ = score_in_sac;
            best_F_21_ = F_21_in_sac;
            is_inlier_match_ = is_inlier_match_in_sac;
        }
    }

    if (0.0 < best_score_) {
        solution_is_valid_ = true;
    }

    if (!recompute || !solution_is_valid_) {
        return;
    }

    // インライアのみでもう一度F行列を求める
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
    const Mat33_t normalized_F_21 = solve::fundamental_solver::compute_F_21(inlier_normalized_keypts_1, inlier_normalized_keypts_2);
    best_F_21_ = transform_2_t * normalized_F_21 * transform_1;
    best_score_ = check_inliers(best_F_21_, is_inlier_match_);
}

float fundamental_solver::check_inliers(const Mat33_t& F_21, std::vector<bool>& is_inlier_match) {
    const auto num_points = matches_12_.size();

    // 自由度1のカイ2乗分布におけるカイ2乗値(上端有意確率0.05)
    // (fundamental transformの誤差は1自由度)
    constexpr float chi_sq_thr = 3.841;
    // 自由度2のカイ2乗分布におけるカイ2乗値(上端有意確率0.05)
    // (FとHのスコアを合わせるために用いる)
    constexpr float score_thr = 5.991;

    is_inlier_match.resize(num_points);

    const Mat33_t F_12 = F_21.transpose();

    float score = 0;

    const float inv_sigma_sq = 1.0 / (sigma_ * sigma_);

    for (unsigned int i = 0; i < num_points; ++i) {
        const auto& keypt_1 = undist_keypts_1_.at(matches_12_.at(i).first);
        const auto& keypt_2 = undist_keypts_2_.at(matches_12_.at(i).second);

        // 1.fundamental変換するため同次座標に変換

        const Vec3_t pt_1 = util::converter::to_homogeneous(keypt_1.pt);
        const Vec3_t pt_2 = util::converter::to_homogeneous(keypt_2.pt);

        // 2. symmetric transfer errorを計算

        // 2-1. 1の点を2上のエピポーラ線に変換し，transfer error(点と線の距離)を計算

        const Vec3_t epiline_in_2 = F_21 * pt_1;

        const float residual_in_2 = epiline_in_2.dot(pt_2);
        const float dist_sq_2 = residual_in_2 * residual_in_2 / epiline_in_2.block<2, 1>(0, 0).squaredNorm();

        // 分散で標準化する
        const float chi_sq_2 = dist_sq_2 * inv_sigma_sq;

        // インライアであればスコアを計算する
        if (chi_sq_thr < chi_sq_2) {
            is_inlier_match.at(i) = false;
            continue;
        }
        else {
            is_inlier_match.at(i) = true;
            score += score_thr - chi_sq_2;
        }

        // 2-2. 2の点を1上のエピポーラ線に変換し，transfer error(点と線の距離)を計算

        const Vec3_t epiline_in_1 = F_12 * pt_2;

        const float residual_in_1 = epiline_in_1.dot(pt_1);
        const float dist_sq_1 = residual_in_1 * residual_in_1 / epiline_in_1.block<2, 1>(0, 0).squaredNorm();

        // 分散で標準化する
        const float chi_sq_1 = dist_sq_1 * inv_sigma_sq;

        // インライアであればスコアを計算する
        if (chi_sq_thr < chi_sq_1) {
            is_inlier_match.at(i) = false;
            continue;
        }
        else {
            is_inlier_match.at(i) = true;
            score += score_thr - chi_sq_1;
        }
    }

    return score;
}

Mat33_t fundamental_solver::compute_F_21(const std::vector<cv::Point2f>& keypts_1, const std::vector<cv::Point2f>& keypts_2) {
    assert(keypts_1.size() == keypts_2.size());

    const auto num_points = keypts_1.size();

    typedef Eigen::Matrix<Mat33_t::Scalar, Eigen::Dynamic, 9> CoeffMatrix;
    CoeffMatrix A(num_points, 9);

    for (unsigned int i = 0; i < num_points; i++) {
        const float x_1 = keypts_1.at(i).x;
        const float y_1 = keypts_1.at(i).y;
        const float x_2 = keypts_2.at(i).x;
        const float y_2 = keypts_2.at(i).y;

        A(i, 0) = x_2 * x_1;
        A(i, 1) = x_2 * y_1;
        A(i, 2) = x_2;
        A(i, 3) = y_2 * x_1;
        A(i, 4) = y_2 * y_1;
        A(i, 5) = y_2;
        A(i, 6) = x_1;
        A(i, 7) = y_1;
        A(i, 8) = 1;
    }

    const Eigen::JacobiSVD<CoeffMatrix> init_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix<Mat33_t::Scalar, 9, 1> v = init_svd.matrixV().col(8);
    // pointerをコンストラクタに渡した場合，col-majorで値が格納されるため，transposeする必要がある
    const Mat33_t init_F_21 = Mat33_t(v.data()).transpose();

    const Eigen::JacobiSVD<Mat33_t> svd(init_F_21, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Mat33_t& U = svd.matrixU();
    Vec3_t lambda = svd.singularValues();
    const Mat33_t& V = svd.matrixV();

    lambda(2) = 0.0;

    const Mat33_t F_21 = U * lambda.asDiagonal() * V.transpose();

    return F_21;
}

bool fundamental_solver::decompose(const Mat33_t& F_21, const Mat33_t& cam_matrix_1, const Mat33_t& cam_matrix_2,
                                   eigen_alloc_vector<Mat33_t>& init_rots, eigen_alloc_vector<Vec3_t>& init_transes) {
    // https://en.wikipedia.org/wiki/Essential_matrix#Determining_R_and_t_from_E

    const Mat33_t E_21 = cam_matrix_2.transpose() * F_21 * cam_matrix_1;

    const Eigen::JacobiSVD<Mat33_t> svd(E_21, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Vec3_t trans = svd.matrixU().col(2);
    trans.normalize();

    Mat33_t W = Mat33_t::Zero();
    W(0, 1) = -1;
    W(1, 0) = 1;
    W(2, 2) = 1;

    Mat33_t rot_1 = svd.matrixU() * W * svd.matrixV().transpose();
    if (rot_1.determinant() < 0) {
        rot_1 *= -1;
    }

    Mat33_t rot_2 = svd.matrixU() * W.transpose() * svd.matrixV().transpose();
    if (rot_2.determinant() < 0) {
        rot_2 *= -1;
    }

    init_rots = {rot_1, rot_1, rot_2, rot_2};
    init_transes = {trans, -trans, trans, -trans};

    return true;
}

} // namespace solver
} // namespace openvslam
