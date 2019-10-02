#include "helper/keypoint.h"
#include "helper/landmark.h"

#include "openvslam/type.h"
#include "openvslam/solve/homography_solver.h"
#include "openvslam/util/converter.h"

#include <gtest/gtest.h>

using namespace openvslam;

TEST(homography_solver, linear_solve) {
    // create 3D points
    const unsigned int num_landmarks = 100;
    const Vec4_t plane_coeffs{-3.0, 5.0, 2.0, 23.0};
    const auto landmarks = create_random_landmarks_on_plane(num_landmarks, 100, plane_coeffs);

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

    // solve with SVD
    const Mat33_t H_21 = solve::homography_solver::compute_H_21(keypts_1, keypts_2);
    const Mat33_t H_12 = H_21.inverse();

    // check symmetric transform error
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        const Vec3_t keypt_1 = util::converter::to_homogeneous(keypts_1.at(i));
        const Vec3_t keypt_2 = util::converter::to_homogeneous(keypts_2.at(i));
        // transform each other
        Vec3_t keypt_1_in_2 = H_21 * keypt_1;
        keypt_1_in_2 /= keypt_1_in_2(2);
        Vec3_t keypt_2_in_1 = H_12 * keypt_2;
        keypt_2_in_1 /= keypt_2_in_1(2);
        // check errors
        EXPECT_LT((keypt_1 - keypt_2_in_1).norm(), 5.0);
        EXPECT_LT((keypt_2 - keypt_1_in_2).norm(), 5.0);
    }
}

TEST(homography_solver, ransac_solve_without_noise) {
    // create 3D points
    const unsigned int num_landmarks = 200;
    const Vec4_t plane_coeffs{4.0, -3.0, 9.0, -34.0};
    const auto landmarks = create_random_landmarks_on_plane(num_landmarks, 100, plane_coeffs);

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

    // create matching information
    std::vector<std::pair<int, int>> matches_12(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        matches_12.at(i).first = i;
        matches_12.at(i).second = i;
    }

    // solve via RANSAC
    solve::homography_solver solver(keypts_1, keypts_2, matches_12, 1.0);
    solver.find_via_ransac(100);
    const Mat33_t H_21 = solver.get_best_H_21();
    const Mat33_t H_12 = H_21.inverse();

    // check that solution is valid
    EXPECT_TRUE(solver.solution_is_valid());

    // check symmetric transform error
    const auto inlier_matches = solver.get_inlier_matches();
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        if (!inlier_matches.at(i)) {
            continue;
        }
        const Vec3_t keypt_1 = util::converter::to_homogeneous(keypts_1.at(i).pt);
        const Vec3_t keypt_2 = util::converter::to_homogeneous(keypts_2.at(i).pt);
        // transform each other
        Vec3_t keypt_1_in_2 = H_21 * keypt_1;
        keypt_1_in_2 /= keypt_1_in_2(2);
        Vec3_t keypt_2_in_1 = H_12 * keypt_2;
        keypt_2_in_1 /= keypt_2_in_1(2);
        // check errors
        EXPECT_LT((keypt_1 - keypt_2_in_1).norm(), 5.0);
        EXPECT_LT((keypt_2 - keypt_1_in_2).norm(), 5.0);
    }
}

TEST(homography_solver, ransac_solve_with_noise) {
    // create 3D points
    const unsigned int num_landmarks = 200;
    const Vec4_t plane_coeffs{-9.0, 2.0, -6.0, 11.0};
    const auto landmarks = create_random_landmarks_on_plane(num_landmarks, 100, plane_coeffs);

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

    // create matching information
    std::vector<std::pair<int, int>> matches_12(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        matches_12.at(i).first = i;
        matches_12.at(i).second = i;
    }

    // solve via RANSAC
    solve::homography_solver solver(keypts_1, keypts_2, matches_12, 1.0);
    solver.find_via_ransac(100);
    const Mat33_t H_21 = solver.get_best_H_21();
    const Mat33_t H_12 = H_21.inverse();

    // check that solution is valid
    EXPECT_TRUE(solver.solution_is_valid());

    // check symmetric transform error
    const auto inlier_matches = solver.get_inlier_matches();
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        if (!inlier_matches.at(i)) {
            continue;
        }
        const Vec3_t keypt_1 = util::converter::to_homogeneous(keypts_1.at(i).pt);
        const Vec3_t keypt_2 = util::converter::to_homogeneous(keypts_2.at(i).pt);
        // transform each other
        Vec3_t keypt_1_in_2 = H_21 * keypt_1;
        keypt_1_in_2 /= keypt_1_in_2(2);
        Vec3_t keypt_2_in_1 = H_12 * keypt_2;
        keypt_2_in_1 /= keypt_2_in_1(2);
        // check errors
        EXPECT_LT((keypt_1 - keypt_2_in_1).norm(), 5.0);
        EXPECT_LT((keypt_2 - keypt_1_in_2).norm(), 5.0);
    }
}
