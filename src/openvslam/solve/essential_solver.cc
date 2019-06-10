#include "openvslam/solve/common.h"
#include "openvslam/solve/essential_solver.h"
#include "openvslam/util/converter.h"
#include "openvslam/util/random_array.h"

namespace openvslam {
namespace solve {

essential_solver::essential_solver(const eigen_alloc_vector<Vec3_t>& bearings_1, const eigen_alloc_vector<Vec3_t>& bearings_2,
                                   const std::vector<std::pair<int, int>>& matches_12)
        : bearings_1_(bearings_1), bearings_2_(bearings_2), matches_12_(matches_12) {}

void essential_solver::find_via_ransac(const unsigned int max_num_iter, const bool recompute) {
    const auto num_matches = static_cast<unsigned int>(matches_12_.size());

    // 1. RANSACを回す

    // 1-1. 変数の準備

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
    eigen_alloc_vector<Vec3_t> min_set_bearings_1(min_set_size);
    eigen_alloc_vector<Vec3_t> min_set_bearings_2(min_set_size);
    // RANSACループ内で使い回す変数
    // E_21_in_sac: 1->2のfundamental
    Mat33_t E_21_in_sac;
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
            min_set_bearings_1.at(i) = bearings_1_.at(matches_12_.at(idx).first);
            min_set_bearings_2.at(i) = bearings_2_.at(matches_12_.at(idx).second);
        }

        // 2-3. E行列を求める
        E_21_in_sac = compute_E_21(min_set_bearings_1, min_set_bearings_2);

        // 2-４. インライアチェック
        score_in_sac = check_inliers(E_21_in_sac, is_inlier_match_in_sac);

        // 2-5. ベストモデルの更新
        if (best_score_ < score_in_sac) {
            best_score_ = score_in_sac;
            best_E_21_ = E_21_in_sac;
            is_inlier_match_ = is_inlier_match_in_sac;
        }
    }

    if (0.0 < best_score_) {
        solution_is_valid_ = true;
    }

    if (!recompute || !solution_is_valid_) {
        return;
    }

    // インライアのみでもう一度E行列を求める
    eigen_alloc_vector<Vec3_t> inlier_bearing_1;
    eigen_alloc_vector<Vec3_t> inlier_bearing_2;
    inlier_bearing_1.reserve(matches_12_.size());
    inlier_bearing_2.reserve(matches_12_.size());
    for (unsigned int i = 0; i < matches_12_.size(); ++i) {
        if (is_inlier_match_.at(i)) {
            inlier_bearing_1.push_back(bearings_1_.at(matches_12_.at(i).first));
            inlier_bearing_2.push_back(bearings_2_.at(matches_12_.at(i).second));
        }
    }

    // スコアをもう一度計算する
    best_E_21_ = solve::essential_solver::compute_E_21(inlier_bearing_1, inlier_bearing_2);
    best_score_ = check_inliers(best_E_21_, is_inlier_match_);
}

float essential_solver::check_inliers(const Mat33_t& E_21, std::vector<bool>& is_inlier_match) {
    const auto num_points = matches_12_.size();

    is_inlier_match.resize(num_points);

    const Mat33_t E_12 = E_21.transpose();

    float score = 0;

    // bearingと法線ベクトルの内積の閾値(=cos(89deg))
    constexpr float residual_cos_thr = 0.01745240643;

    for (unsigned int i = 0; i < num_points; ++i) {
        const auto& bearing_1 = bearings_1_.at(matches_12_.at(i).first);
        const auto& bearing_2 = bearings_2_.at(matches_12_.at(i).second);

        // 1. symmetric transfer errorを計算

        // 1-1. 1の点を2上のエピポーラ平面に変換し，transfer error(bearingと法線ベクトルの内積)を計算

        const Vec3_t epiplane_in_2 = E_21 * bearing_1;

        const float residual_in_2 = std::abs(epiplane_in_2.dot(bearing_2) / epiplane_in_2.norm());

        // インライアであればスコアを計算する
        if (residual_cos_thr < residual_in_2) {
            is_inlier_match.at(i) = false;
            continue;
        }
        else {
            is_inlier_match.at(i) = true;
            score += residual_in_2;
        }

        // 2-2. 2の点を1上のエピポーラ平面に変換し，transfer error(bearingと法線ベクトルの内積)を計算

        const Vec3_t epiplane_in_1 = E_12 * bearing_2;

        const float residual_in_1 = std::abs(epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm());

        // インライアであればスコアを計算する
        if (residual_cos_thr < residual_in_1) {
            is_inlier_match.at(i) = false;
            continue;
        }
        else {
            is_inlier_match.at(i) = true;
            score += residual_in_1;
        }
    }

    return score;
}

Mat33_t essential_solver::compute_E_21(const eigen_alloc_vector<Vec3_t>& bearings_1, const eigen_alloc_vector<Vec3_t>& bearings_2) {
    assert(bearings_1.size() == bearings_2.size());

    const auto num_points = bearings_1.size();

    typedef Eigen::Matrix<Mat33_t::Scalar, Eigen::Dynamic, 9> CoeffMatrix;
    CoeffMatrix A(num_points, 9);

    for (unsigned int i = 0; i < num_points; i++) {
        A.block<1, 3>(i, 0) = bearings_2.at(i)(0) * bearings_1.at(i);
        A.block<1, 3>(i, 3) = bearings_2.at(i)(1) * bearings_1.at(i);
        A.block<1, 3>(i, 6) = bearings_2.at(i)(2) * bearings_1.at(i);
    }

    const Eigen::JacobiSVD<CoeffMatrix> init_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix<Mat33_t::Scalar, 9, 1> v = init_svd.matrixV().col(8);
    // pointerをコンストラクタに渡した場合，col-majorで値が格納されるため，transposeする必要がある
    const Mat33_t init_E_21 = Mat33_t(v.data()).transpose();

    const Eigen::JacobiSVD<Mat33_t> svd(init_E_21, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Mat33_t& U = svd.matrixU();
    Vec3_t lambda = svd.singularValues();
    const Mat33_t& V = svd.matrixV();

    lambda(2) = 0.0;

    const Mat33_t E_21 = U * lambda.asDiagonal() * V.transpose();

    return E_21;
}

bool essential_solver::decompose(const Mat33_t& E_21, eigen_alloc_vector<Mat33_t>& init_rots, eigen_alloc_vector<Vec3_t>& init_transes) {
    // https://en.wikipedia.org/wiki/Essential_matrix#Determining_R_and_t_from_E

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
