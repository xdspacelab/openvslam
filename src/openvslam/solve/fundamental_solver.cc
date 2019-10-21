#include "openvslam/solve/common.h"
#include "openvslam/solve/essential_solver.h"
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

    // 0. Normalize keypoint coordinates

    // apply normalization
    std::vector<cv::Point2f> normalized_keypts_1, normalized_keypts_2;
    Mat33_t transform_1, transform_2;
    normalize(undist_keypts_1_, normalized_keypts_1, transform_1);
    normalize(undist_keypts_2_, normalized_keypts_2, transform_2);

    const Mat33_t transform_2_t = transform_2.transpose();

    // 1. Prepare for RANSAC

    // minimum number of samples (= 8)
    constexpr unsigned int min_set_size = 8;
    if (num_matches < min_set_size) {
        solution_is_valid_ = false;
        return;
    }

    // RANSAC variables
    best_score_ = 0.0;
    is_inlier_match_ = std::vector<bool>(num_matches, false);

    // minimum set of keypoint matches
    std::vector<cv::Point2f> min_set_keypts_1(min_set_size);
    std::vector<cv::Point2f> min_set_keypts_2(min_set_size);

    // shared variables in RANSAC loop
    // fundamental matrix from shot 1 to shot 2
    Mat33_t F_21_in_sac;
    // inlier/outlier flags
    std::vector<bool> is_inlier_match_in_sac(num_matches, false);
    // score of fundamental matrix
    float score_in_sac;

    // 2. RANSAC loop

    for (unsigned int iter = 0; iter < max_num_iter; iter++) {
        // 2-1. Create a minimum set
        const auto indices = util::create_random_array(min_set_size, 0U, num_matches - 1);
        for (unsigned int i = 0; i < min_set_size; ++i) {
            const auto idx = indices.at(i);
            min_set_keypts_1.at(i) = normalized_keypts_1.at(matches_12_.at(idx).first);
            min_set_keypts_2.at(i) = normalized_keypts_2.at(matches_12_.at(idx).second);
        }

        // 2-2. Compute a fundamental matrix
        const Mat33_t normalized_F_21 = compute_F_21(min_set_keypts_1, min_set_keypts_2);
        F_21_in_sac = transform_2_t * normalized_F_21 * transform_1;

        // 2-3. Check inliers and compute a score
        score_in_sac = check_inliers(F_21_in_sac, is_inlier_match_in_sac);

        // 2-4. Update the best model
        if (best_score_ < score_in_sac) {
            best_score_ = score_in_sac;
            best_F_21_ = F_21_in_sac;
            is_inlier_match_ = is_inlier_match_in_sac;
        }
    }

    const auto num_inliers = std::count(is_inlier_match_.begin(), is_inlier_match_.end(), true);
    solution_is_valid_ = (best_score_ > 0.0) && (num_inliers >= min_set_size);

    if (!recompute || !solution_is_valid_) {
        return;
    }

    // 3. Recompute a fundamental matrix only with the inlier matches

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
    const Mat33_t normalized_F_21 = solve::fundamental_solver::compute_F_21(inlier_normalized_keypts_1, inlier_normalized_keypts_2);
    best_F_21_ = transform_2_t * normalized_F_21 * transform_1;
    best_score_ = check_inliers(best_F_21_, is_inlier_match_);
}

Mat33_t fundamental_solver::compute_F_21(const std::vector<cv::Point2f>& keypts_1, const std::vector<cv::Point2f>& keypts_2) {
    assert(keypts_1.size() == keypts_2.size());

    const auto num_points = keypts_1.size();

    typedef Eigen::Matrix<Mat33_t::Scalar, Eigen::Dynamic, 9> CoeffMatrix;
    CoeffMatrix A(num_points, 9);

    for (unsigned int i = 0; i < num_points; i++) {
        A.block<1, 3>(i, 0) = keypts_2.at(i).x * util::converter::to_homogeneous(keypts_1.at(i));
        A.block<1, 3>(i, 3) = keypts_2.at(i).y * util::converter::to_homogeneous(keypts_1.at(i));
        A.block<1, 3>(i, 6) = util::converter::to_homogeneous(keypts_1.at(i));
    }

    const Eigen::JacobiSVD<CoeffMatrix> init_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix<Mat33_t::Scalar, 9, 1> v = init_svd.matrixV().col(8);
    // need transpose() because elements are contained as col-major after it was constructed from a pointer
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
    const Mat33_t E_21 = cam_matrix_2.transpose() * F_21 * cam_matrix_1;
    essential_solver::decompose(E_21, init_rots, init_transes);
    return true;
}

Mat33_t fundamental_solver::create_F_21(const Mat33_t& rot_1w, const Vec3_t& trans_1w, const Mat33_t& cam_matrix_1,
                                        const Mat33_t& rot_2w, const Vec3_t& trans_2w, const Mat33_t& cam_matrix_2) {
    const Mat33_t E_21 = essential_solver::create_E_21(rot_1w, trans_1w, rot_2w, trans_2w);
    return cam_matrix_2.transpose().inverse() * E_21 * cam_matrix_1.inverse();
}

float fundamental_solver::check_inliers(const Mat33_t& F_21, std::vector<bool>& is_inlier_match) {
    const auto num_points = matches_12_.size();

    // chi-squared value (p=0.05, n=1)
    constexpr float chi_sq_thr = 3.841;
    // chi-squared value (p=0.05, n=2)
    constexpr float score_thr = 5.991;

    is_inlier_match.resize(num_points);

    const Mat33_t F_12 = F_21.transpose();

    float score = 0;

    const float inv_sigma_sq = 1.0 / (sigma_ * sigma_);

    for (unsigned int i = 0; i < num_points; ++i) {
        const auto& keypt_1 = undist_keypts_1_.at(matches_12_.at(i).first);
        const auto& keypt_2 = undist_keypts_2_.at(matches_12_.at(i).second);

        // 1. Transform to homogeneous coordinates

        const Vec3_t pt_1 = util::converter::to_homogeneous(keypt_1.pt);
        const Vec3_t pt_2 = util::converter::to_homogeneous(keypt_2.pt);

        // 2. Compute symmetric transfer error

        // 2-1. Transform a point in shot 1 to the epipolar line in shot 2,
        //      then compute a transfer error (= dot product)

        const Vec3_t epiline_in_2 = F_21 * pt_1;

        const float residual_in_2 = epiline_in_2.dot(pt_2);
        const float dist_sq_2 = residual_in_2 * residual_in_2 / epiline_in_2.block<2, 1>(0, 0).squaredNorm();

        // standardization
        const float chi_sq_2 = dist_sq_2 * inv_sigma_sq;

        // if a match is inlier, accumulate the score
        if (chi_sq_thr < chi_sq_2) {
            is_inlier_match.at(i) = false;
            continue;
        }
        else {
            is_inlier_match.at(i) = true;
            score += score_thr - chi_sq_2;
        }

        // 2. Transform a point in shot 2 to the epipolar line in shot 1,
        //    then compute a transfer error (= dot product)

        const Vec3_t epiline_in_1 = F_12 * pt_2;

        const float residual_in_1 = epiline_in_1.dot(pt_1);
        const float dist_sq_1 = residual_in_1 * residual_in_1 / epiline_in_1.block<2, 1>(0, 0).squaredNorm();

        // standardization
        const float chi_sq_1 = dist_sq_1 * inv_sigma_sq;

        // if a match is inlier, accumulate the score
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

} // namespace solve
} // namespace openvslam
