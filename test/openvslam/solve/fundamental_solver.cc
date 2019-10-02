#include "helper/keypoint.h"
#include "helper/landmark.h"

#include "openvslam/type.h"
#include "openvslam/solve/fundamental_solver.h"
#include "openvslam/util/converter.h"

#include <gtest/gtest.h>

using namespace openvslam;

TEST(fundamental_solver, linear_solve) {
    // create 3D points
    const unsigned int num_landmarks = 100;
    const auto landmarks = create_random_landmarks_in_space(num_landmarks, 100);

    // create two-view poses
    const Mat33_t rot_1 = util::converter::to_rot_mat(205.0 * M_PI / 180.0 * Vec3_t{4, -6, 2}.normalized());
    const Vec3_t trans_1 = Vec3_t(-28.1, -63.3, 43.4);
    const Mat33_t rot_2 = util::converter::to_rot_mat(-15.0 * M_PI / 180.0 * Vec3_t{5, 1, -3}.normalized());
    const Vec3_t trans_2 = Vec3_t(-30.4, -45.5, -49.6);
    // create camera matrices
    Mat33_t cam_matrix_1 = Mat33_t::Identity();
    cam_matrix_1(0, 0) = 143.0;
    cam_matrix_1(0, 2) = 400.0;
    cam_matrix_1(1, 1) = 140.0;
    cam_matrix_1(1, 2) = 700.0;
    Mat33_t cam_matrix_2 = Mat33_t::Identity();
    cam_matrix_2(0, 0) = 113.0;
    cam_matrix_2(0, 2) = 200.0;
    cam_matrix_2(1, 1) = 108.0;
    cam_matrix_2(1, 2) = 500.0;

    // create keypoints from two-view poses and 3D points
    std::vector<cv::Point2f> keypts_1;
    std::vector<cv::Point2f> keypts_2;
    create_keypoints(rot_1, trans_1, cam_matrix_1, landmarks, keypts_1);
    create_keypoints(rot_2, trans_2, cam_matrix_2, landmarks, keypts_2);

    // create a true fundamental matrix
    Mat33_t true_F_21 = solve::fundamental_solver::create_F_21(rot_1, trans_1, cam_matrix_1, rot_2, trans_2, cam_matrix_2);

    // solve with SVD
    Mat33_t F_21 = solve::fundamental_solver::compute_F_21(keypts_1, keypts_2);

    // align scale and sign
    true_F_21 /= true_F_21.norm();
    F_21 /= F_21.norm();
    if (true_F_21(0, 0) * F_21(0, 0) < 0) {
        true_F_21 *= -1.0;
    }

    EXPECT_LT((true_F_21 - F_21).norm(), 1e-4);
}

TEST(fundamental_solver, ransac_solve_without_noise) {
    // create 3D points
    const unsigned int num_landmarks = 200;
    const auto landmarks = create_random_landmarks_in_space(num_landmarks, 100);

    // create two-view poses
    const Mat33_t rot_1 = util::converter::to_rot_mat(-105.0 * M_PI / 180.0 * Vec3_t{1, 10, 3}.normalized());
    const Vec3_t trans_1 = Vec3_t(-49.1, -25.3, -3.4);
    const Mat33_t rot_2 = util::converter::to_rot_mat(275.0 * M_PI / 180.0 * Vec3_t{-5, 5, -4}.normalized());
    const Vec3_t trans_2 = Vec3_t(20.4, 25.5, 39.6);
    // create camera matrices
    Mat33_t cam_matrix_1 = Mat33_t::Identity();
    cam_matrix_1(0, 0) = 103.0;
    cam_matrix_1(0, 2) = 250.0;
    cam_matrix_1(1, 1) = 101.0;
    cam_matrix_1(1, 2) = 300.0;
    Mat33_t cam_matrix_2 = Mat33_t::Identity();
    cam_matrix_2(0, 0) = 123.0;
    cam_matrix_2(0, 2) = 300.0;
    cam_matrix_2(1, 1) = 118.0;
    cam_matrix_2(1, 2) = 400.0;

    // create keypoints from two-view poses and 3D points
    std::vector<cv::KeyPoint> keypts_1;
    std::vector<cv::KeyPoint> keypts_2;
    create_keypoints(rot_1, trans_1, cam_matrix_1, landmarks, keypts_1);
    create_keypoints(rot_2, trans_2, cam_matrix_2, landmarks, keypts_2);

    // create a true fundamental matrix
    Mat33_t true_F_21 = solve::fundamental_solver::create_F_21(rot_1, trans_1, cam_matrix_1, rot_2, trans_2, cam_matrix_2);

    // create matching information
    std::vector<std::pair<int, int>> matches_12(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        matches_12.at(i).first = i;
        matches_12.at(i).second = i;
    }

    // solve via RANSAC
    solve::fundamental_solver solver(keypts_1, keypts_2, matches_12, 1.0);
    solver.find_via_ransac(100);
    Mat33_t F_21 = solver.get_best_F_21();

    // check that solution is valid
    EXPECT_TRUE(solver.solution_is_valid());

    // check that all of the matches are inlier
    const auto inlier_matches = solver.get_inlier_matches();
    EXPECT_TRUE(std::all_of(inlier_matches.begin(), inlier_matches.end(),
                            [](const bool is_inlier) { return is_inlier; }));

    // align scale and sign
    true_F_21 /= true_F_21.norm();
    F_21 /= F_21.norm();
    if (true_F_21(0, 0) * F_21(0, 0) < 0) {
        true_F_21 *= -1.0;
    }

    EXPECT_LT((true_F_21 - F_21).norm(), 1e-4);
}

TEST(fundamental_solver, ransac_solve_with_noise) {
    // create 3D points
    const unsigned int num_landmarks = 200;
    const auto landmarks = create_random_landmarks_in_space(num_landmarks, 100);

    // create two-view poses
    const Mat33_t rot_1 = util::converter::to_rot_mat(54.0 * M_PI / 180.0 * Vec3_t{5, 3, -2}.normalized());
    const Vec3_t trans_1 = Vec3_t(40.3, -31.6, 58.4);
    const Mat33_t rot_2 = util::converter::to_rot_mat(-21.0 * M_PI / 180.0 * Vec3_t{-2, -5, 6}.normalized());
    const Vec3_t trans_2 = Vec3_t(-45.4, 11.5, -24.6);
    // create camera matrices
    Mat33_t cam_matrix_1 = Mat33_t::Identity();
    cam_matrix_1(0, 0) = 233.0;
    cam_matrix_1(0, 2) = 800.0;
    cam_matrix_1(1, 1) = 220.0;
    cam_matrix_1(1, 2) = 600.0;
    Mat33_t cam_matrix_2 = Mat33_t::Identity();
    cam_matrix_2(0, 0) = 103.0;
    cam_matrix_2(0, 2) = 400.0;
    cam_matrix_2(1, 1) = 108.0;
    cam_matrix_2(1, 2) = 300.0;

    // create keypoints from two-view poses and 3D points
    std::vector<cv::KeyPoint> keypts_1;
    std::vector<cv::KeyPoint> keypts_2;
    create_keypoints(rot_1, trans_1, cam_matrix_1, landmarks, keypts_1, 1.0);
    create_keypoints(rot_2, trans_2, cam_matrix_2, landmarks, keypts_2, 1.0);

    // create a true fundamental matrix
    Mat33_t true_F_21 = solve::fundamental_solver::create_F_21(rot_1, trans_1, cam_matrix_1, rot_2, trans_2, cam_matrix_2);

    // create matching information
    std::vector<std::pair<int, int>> matches_12(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        matches_12.at(i).first = i;
        matches_12.at(i).second = i;
    }

    // solve via RANSAC
    solve::fundamental_solver solver(keypts_1, keypts_2, matches_12, 1.0);
    solver.find_via_ransac(100);
    Mat33_t F_21 = solver.get_best_F_21();

    // check that solution is valid
    EXPECT_TRUE(solver.solution_is_valid());

    // align scale and sign
    true_F_21 /= true_F_21.norm();
    F_21 /= F_21.norm();
    if (true_F_21(0, 0) * F_21(0, 0) < 0) {
        true_F_21 *= -1.0;
    }

    EXPECT_LT((true_F_21 - F_21).norm(), 1e-2);
}

TEST(fundamental_solver, decompose) {
    // create two-view poses
    const Mat33_t rot_1 = util::converter::to_rot_mat(205.0 * M_PI / 180.0 * Vec3_t{4, -6, 2}.normalized());
    const Vec3_t trans_1 = Vec3_t(-28.1, -63.3, 43.4);
    const Mat33_t rot_2 = util::converter::to_rot_mat(-15.0 * M_PI / 180.0 * Vec3_t{5, 1, -3}.normalized());
    const Vec3_t trans_2 = Vec3_t(-30.4, -45.5, -49.6);
    // create camera matrices
    Mat33_t cam_matrix_1 = Mat33_t::Identity();
    cam_matrix_1(0, 0) = 231.0;
    cam_matrix_1(0, 2) = 500.0;
    cam_matrix_1(1, 1) = 229.0;
    cam_matrix_1(1, 2) = 1000.0;
    Mat33_t cam_matrix_2 = Mat33_t::Identity();
    cam_matrix_2(0, 0) = 104.0;
    cam_matrix_2(0, 2) = 120.0;
    cam_matrix_2(1, 1) = 101.0;
    cam_matrix_2(1, 2) = 240.0;

    // create a true fundamental matrix
    Mat33_t true_F_21 = solve::fundamental_solver::create_F_21(rot_1, trans_1, cam_matrix_1, rot_2, trans_2, cam_matrix_2);

    // decompose
    eigen_alloc_vector<Mat33_t> rots;
    eigen_alloc_vector<Vec3_t> transes;
    solve::fundamental_solver::decompose(true_F_21, cam_matrix_1, cam_matrix_2, rots, transes);
    EXPECT_EQ(rots.size(), 4);
    EXPECT_EQ(transes.size(), 4);

    // check one of the hypotheses is match to the true pose
    const Mat33_t rot_21 = rot_2 * rot_1.inverse();
    const Vec3_t trans_21 = trans_2 - rot_21 * trans_1;
    bool matched = false;
    for (unsigned int i = 0; i < 4; ++i) {
        if ((rot_21 - rots.at(i)).norm() < 1e-4) {
            if ((trans_21.normalized() - transes.at(i).normalized()).norm() < 1e-4) {
                matched = true;
            }
        }
    }
    EXPECT_TRUE(matched);
}

TEST(fundamental_solver, create) {
    // create two-view poses
    const Mat33_t rot_1 = util::converter::to_rot_mat(205.0 * M_PI / 180.0 * Vec3_t{4, -6, 2}.normalized());
    const Vec3_t trans_1 = Vec3_t(-28.1, -63.3, 43.4);
    const Mat33_t rot_2 = util::converter::to_rot_mat(-15.0 * M_PI / 180.0 * Vec3_t{5, 1, -3}.normalized());
    const Vec3_t trans_2 = Vec3_t(-30.4, -45.5, -49.6);
    // create camera matrices
    Mat33_t cam_matrix_1 = Mat33_t::Identity();
    cam_matrix_1(0, 0) = 263.0;
    cam_matrix_1(0, 2) = 400.0;
    cam_matrix_1(1, 1) = 250.0;
    cam_matrix_1(1, 2) = 600.0;
    Mat33_t cam_matrix_2 = Mat33_t::Identity();
    cam_matrix_2(0, 0) = 143.0;
    cam_matrix_2(0, 2) = 300.0;
    cam_matrix_2(1, 1) = 148.0;
    cam_matrix_2(1, 2) = 300.0;

    // create a true fundamental matrix
    const Mat33_t rot_21 = rot_2 * rot_1.inverse();
    const Vec3_t trans_21 = trans_2 - rot_21 * trans_1;
    Mat33_t skew_21;
    skew_21 << 0, -trans_21(2), trans_21(1),
        trans_21(2), 0, -trans_21(0),
        -trans_21(1), trans_21(0), 0;
    Mat33_t true_F_21 = cam_matrix_2.transpose().inverse() * skew_21 * rot_21 * cam_matrix_1.inverse();

    // create a fundamental matrix from the rotation and the translation
    const Mat33_t F_21 = solve::fundamental_solver::create_F_21(rot_1, trans_1, cam_matrix_1, rot_2, trans_2, cam_matrix_2);

    EXPECT_LT((true_F_21 - F_21).norm(), 1e-4);
}
