#include "helper/bearing_vector.h"

#include "openvslam/type.h"
#include "openvslam/solve/essential_solver.h"
#include "openvslam/util/random_array.h"

#include <Eigen/Geometry>

#include <gtest/gtest.h>

using namespace openvslam;

TEST(essential_solver, check_E_solution_1) {
    // 3次元点を作成
    const unsigned int num_landmarks = 1000;
    const auto landmarks = create_random_landmarks(num_landmarks);

    // 姿勢を作成
    const Mat33_t rot_1 = Eigen::AngleAxisd(54.0 * M_PI / 180, Vec3_t(5, 3, -2).normalized()).toRotationMatrix();
    const Vec3_t trans_1 = Vec3_t(10.3, -1.6, 8.4);
    const Mat33_t rot_2 = Eigen::AngleAxisd(-21.0 * M_PI / 180, Vec3_t(-2, -5, 6).normalized()).toRotationMatrix();
    const Vec3_t trans_2 = Vec3_t(-5.4, 11.5, -24.6);

    // bearing vectorsを作成
    Mat33_t true_E_21;
    eigen_alloc_vector<Vec3_t> bearings_1;
    eigen_alloc_vector<Vec3_t> bearings_2;
    create_bearing_vectors(rot_1, trans_1, rot_2, trans_2, landmarks, true_E_21, bearings_1, bearings_2);

    // matchesを作成
    std::vector<std::pair<int, int>> matches_12(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        matches_12.at(i).first = i;
        matches_12.at(i).second = i;
    }

    // E行列を求める
    solve::essential_solver solver(bearings_1, bearings_2, matches_12);
    solver.find_via_ransac(100);
    Mat33_t E_21 = solver.get_best_E_21();

    // スケールと正負を合わせる
    true_E_21 /= true_E_21.norm();
    E_21 /= E_21.norm();
    if (true_E_21.mean() * E_21.mean() < 0) {
        true_E_21 *= -1.0;
    }

    EXPECT_LT((true_E_21 - E_21).norm(), 1e-4);
}

TEST(essential_solver, check_E_solution_2) {
    // 3次元点を作成
    const unsigned int num_landmarks = 1000;
    const auto landmarks = create_random_landmarks(num_landmarks);

    // 姿勢を作成
    const Mat33_t rot_1 = Eigen::AngleAxisd(54.0 * M_PI / 180, Vec3_t(5, 3, -2).normalized()).toRotationMatrix();
    const Vec3_t trans_1 = Vec3_t(10.3, -1.6, 8.4);
    const Mat33_t rot_2 = Eigen::AngleAxisd(-21.0 * M_PI / 180, Vec3_t(-2, -5, 6).normalized()).toRotationMatrix();
    const Vec3_t trans_2 = Vec3_t(-5.4, 11.5, -24.6);

    // bearing vectorsを作成
    Mat33_t true_E_21;
    eigen_alloc_vector<Vec3_t> bearings_1;
    eigen_alloc_vector<Vec3_t> bearings_2;
    create_bearing_vectors(rot_1, trans_1, rot_2, trans_2, landmarks, true_E_21, bearings_1, bearings_2, 0.01);

    // matchesを作成
    std::vector<std::pair<int, int>> matches_12(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        matches_12.at(i).first = i;
        matches_12.at(i).second = i;
    }

    // E行列を求める
    solve::essential_solver solver(bearings_1, bearings_2, matches_12);
    solver.find_via_ransac(100);
    Mat33_t E_21 = solver.get_best_E_21();

    // スケールと正負を合わせる
    true_E_21 /= true_E_21.norm();
    E_21 /= E_21.norm();
    if (true_E_21.mean() * E_21.mean() < 0) {
        true_E_21 *= -1.0;
    }

    EXPECT_LT((true_E_21 - E_21).norm(), 1e-1);
}
