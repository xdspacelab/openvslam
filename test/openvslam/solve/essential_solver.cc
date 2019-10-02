#include "helper/bearing_vector.h"
#include "helper/landmark.h"

#include "openvslam/type.h"
#include "openvslam/solve/essential_solver.h"
#include "openvslam/util/converter.h"

#include <gtest/gtest.h>

using namespace openvslam;

TEST(essential_solver, linear_solve) {
    // create 3D points
    const unsigned int num_landmarks = 100;
    const auto landmarks = create_random_landmarks_in_space(num_landmarks, 100);

    // create two-view poses
    const Mat33_t rot_1 = util::converter::to_rot_mat(205.0 * M_PI / 180.0 * Vec3_t{4, -6, 2}.normalized());
    const Vec3_t trans_1 = Vec3_t(-28.1, -63.3, 43.4);
    const Mat33_t rot_2 = util::converter::to_rot_mat(-15.0 * M_PI / 180.0 * Vec3_t{5, 1, -3}.normalized());
    const Vec3_t trans_2 = Vec3_t(-30.4, -45.5, -49.6);

    // create bearing vectors from two-view poses and 3D points
    eigen_alloc_vector<Vec3_t> bearings_1;
    eigen_alloc_vector<Vec3_t> bearings_2;
    create_bearing_vectors(rot_1, trans_1, landmarks, bearings_1);
    create_bearing_vectors(rot_2, trans_2, landmarks, bearings_2);

    // create a true essential matrix
    Mat33_t true_E_21 = solve::essential_solver::create_E_21(rot_1, trans_1, rot_2, trans_2);

    // solve with SVD
    Mat33_t E_21 = solve::essential_solver::compute_E_21(bearings_1, bearings_2);

    // align scale and sign
    true_E_21 /= true_E_21.norm();
    E_21 /= E_21.norm();
    if (true_E_21(0, 0) * E_21(0, 0) < 0) {
        true_E_21 *= -1.0;
    }

    EXPECT_LT((true_E_21 - E_21).norm(), 1e-4);
}

TEST(essential_solver, ransac_solve_without_noise) {
    // create 3D points
    const unsigned int num_landmarks = 200;
    const auto landmarks = create_random_landmarks_in_space(num_landmarks, 100);

    // create two-view poses
    const Mat33_t rot_1 = util::converter::to_rot_mat(-105.0 * M_PI / 180.0 * Vec3_t{1, 10, 3}.normalized());
    const Vec3_t trans_1 = Vec3_t(-49.1, -25.3, -3.4);
    const Mat33_t rot_2 = util::converter::to_rot_mat(275.0 * M_PI / 180.0 * Vec3_t{-5, 5, -4}.normalized());
    const Vec3_t trans_2 = Vec3_t(20.4, 25.5, 39.6);

    // create bearing vectors from two-view poses and 3D points
    eigen_alloc_vector<Vec3_t> bearings_1;
    eigen_alloc_vector<Vec3_t> bearings_2;
    create_bearing_vectors(rot_1, trans_1, landmarks, bearings_1);
    create_bearing_vectors(rot_2, trans_2, landmarks, bearings_2);

    // create a true essential matrix
    Mat33_t true_E_21 = solve::essential_solver::create_E_21(rot_1, trans_1, rot_2, trans_2);

    // create matching information
    std::vector<std::pair<int, int>> matches_12(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        matches_12.at(i).first = i;
        matches_12.at(i).second = i;
    }

    // solve via RANSAC
    solve::essential_solver solver(bearings_1, bearings_2, matches_12);
    solver.find_via_ransac(100);
    Mat33_t E_21 = solver.get_best_E_21();

    // check that solution is valid
    EXPECT_TRUE(solver.solution_is_valid());

    // check that all of the matches are inlier
    const auto inlier_matches = solver.get_inlier_matches();
    EXPECT_TRUE(std::all_of(inlier_matches.begin(), inlier_matches.end(),
                            [](const bool is_inlier) { return is_inlier; }));

    // align scale and sign
    true_E_21 /= true_E_21.norm();
    E_21 /= E_21.norm();
    if (true_E_21(0, 0) * E_21(0, 0) < 0) {
        true_E_21 *= -1.0;
    }

    EXPECT_LT((true_E_21 - E_21).norm(), 1e-4);
}

TEST(essential_solver, ransac_solve_with_noise) {
    // create 3D points
    const unsigned int num_landmarks = 200;
    const auto landmarks = create_random_landmarks_in_space(num_landmarks, 100);

    // create two-view poses
    const Mat33_t rot_1 = util::converter::to_rot_mat(54.0 * M_PI / 180.0 * Vec3_t{5, 3, -2}.normalized());
    const Vec3_t trans_1 = Vec3_t(40.3, -31.6, 58.4);
    const Mat33_t rot_2 = util::converter::to_rot_mat(-21.0 * M_PI / 180.0 * Vec3_t{-2, -5, 6}.normalized());
    const Vec3_t trans_2 = Vec3_t(-45.4, 11.5, -24.6);

    // create bearing vectors from two-view poses and 3D points
    eigen_alloc_vector<Vec3_t> bearings_1;
    eigen_alloc_vector<Vec3_t> bearings_2;
    create_bearing_vectors(rot_1, trans_1, landmarks, bearings_1, 0.001);
    create_bearing_vectors(rot_2, trans_2, landmarks, bearings_2, 0.001);

    // create a true essential matrix
    Mat33_t true_E_21 = solve::essential_solver::create_E_21(rot_1, trans_1, rot_2, trans_2);

    // create matching information
    std::vector<std::pair<int, int>> matches_12(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        matches_12.at(i).first = i;
        matches_12.at(i).second = i;
    }

    // solve via RANSAC
    solve::essential_solver solver(bearings_1, bearings_2, matches_12);
    solver.find_via_ransac(100);
    Mat33_t E_21 = solver.get_best_E_21();

    // check that solution is valid
    EXPECT_TRUE(solver.solution_is_valid());

    // align scale and sign
    true_E_21 /= true_E_21.norm();
    E_21 /= E_21.norm();
    if (true_E_21(0, 0) * E_21(0, 0) < 0) {
        true_E_21 *= -1.0;
    }

    EXPECT_LT((true_E_21 - E_21).norm(), 1e-2);
}

TEST(essential_solver, decompose) {
    // create two-view poses
    const Mat33_t rot_1 = util::converter::to_rot_mat(205.0 * M_PI / 180.0 * Vec3_t{4, -6, 2}.normalized());
    const Vec3_t trans_1 = Vec3_t(-28.1, -63.3, 43.4);
    const Mat33_t rot_2 = util::converter::to_rot_mat(-15.0 * M_PI / 180.0 * Vec3_t{5, 1, -3}.normalized());
    const Vec3_t trans_2 = Vec3_t(-30.4, -45.5, -49.6);

    // create a true essential matrix
    Mat33_t true_E_21 = solve::essential_solver::create_E_21(rot_1, trans_1, rot_2, trans_2);

    // decompose
    eigen_alloc_vector<Mat33_t> rots;
    eigen_alloc_vector<Vec3_t> transes;
    solve::essential_solver::decompose(true_E_21, rots, transes);
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

TEST(essential_solver, create) {
    // create two-view poses
    const Mat33_t rot_1 = util::converter::to_rot_mat(205.0 * M_PI / 180.0 * Vec3_t{4, -6, 2}.normalized());
    const Vec3_t trans_1 = Vec3_t(-28.1, -63.3, 43.4);
    const Mat33_t rot_2 = util::converter::to_rot_mat(-15.0 * M_PI / 180.0 * Vec3_t{5, 1, -3}.normalized());
    const Vec3_t trans_2 = Vec3_t(-30.4, -45.5, -49.6);

    // create a true essential matrix
    const Mat33_t rot_21 = rot_2 * rot_1.inverse();
    const Vec3_t trans_21 = trans_2 - rot_21 * trans_1;
    Mat33_t skew_21;
    skew_21 << 0, -trans_21(2), trans_21(1),
        trans_21(2), 0, -trans_21(0),
        -trans_21(1), trans_21(0), 0;
    Mat33_t true_E_21 = skew_21 * rot_21;

    // create an essential matrix from the rotation and the translation
    const Mat33_t E_21 = solve::essential_solver::create_E_21(rot_1, trans_1, rot_2, trans_2);

    EXPECT_LT((true_E_21 - E_21).norm(), 1e-4);
}
