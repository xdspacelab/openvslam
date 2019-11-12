#include "helper/landmark.h"
#include "helper/bearing_vector.h"
#include "openvslam/util/converter.h"
#include "openvslam/solve/pnp_solver.h"
#include "openvslam/type.h"
#include <gtest/gtest.h>
#include <opencv2/core/types.hpp>
#include <memory>
#include <algorithm>
#include <iostream>

using namespace openvslam;

double clamp(const double x, const double min_val, const double max_val) {
    assert(min_val < max_val);
    if (x > max_val)
        return max_val;
    else if (x < min_val)
        return min_val;
    else
        return x;
}
double angle_of_rot(const Mat33_t& rot) {
    const auto trace = rot.trace(); // trace = 1+2cos(θ)
    const auto cos_th = (trace - 1) / 2;
    return acos(clamp(cos_th, -1, 1));
}

TEST(pnp_solver, minimal_case) {
    // Create three landmarks
    const unsigned int num_landmarks = 4;
    eigen_alloc_vector<Vec3_t> landmarks;
    landmarks.emplace_back(Vec3_t{-86.896468, 1.507532, -51.898997});
    landmarks.emplace_back(Vec3_t{-41.515723, -19.622673, 96.996297});
    landmarks.emplace_back(Vec3_t{9.866085, -60.769661, -29.543698});
    landmarks.emplace_back(Vec3_t{-60.437428, -40.322507, 13.686458});

    // Create single-view pose
    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    // Create bearing vectors from pose and landmarks
    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, landmarks, bearings);

    std::vector<cv::KeyPoint> keypts;
    std::vector<float> scale_factors;
    for (unsigned int i = 0; i < num_landmarks; i++) {
        cv::KeyPoint kp;
        keypts.push_back(kp);
        scale_factors.emplace_back(0);
    }

    auto solver = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(bearings, keypts, landmarks, scale_factors, 0));
    solver->find_via_ransac(1);
    EXPECT_TRUE(solver->solution_is_valid());

    const auto estimated_pose = solver->get_best_cam_pose();

    const auto rot = estimated_pose.block<3, 3>(0, 0);
    const auto trans = estimated_pose.block<3, 1>(0, 3);

    const auto rot_err = angle_of_rot(rot_gt * rot.transpose());
    const auto trans_err = (trans_gt - trans).norm();

    EXPECT_LT(rot_err, 1e-2);
    EXPECT_LT(trans_err, 1e-1);
}

TEST(pnp_solver, without_noise) {
    // Create three landmarks
    const unsigned int num_landmarks = 100;
    const auto landmarks = create_random_landmarks_in_space(num_landmarks, 100);

    // Create single-view pose
    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    // Create bearing vectors from pose and landmarks
    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, landmarks, bearings);

    // Make dummy keypoints
    std::vector<cv::KeyPoint> keypts;
    std::vector<float> scale_factors;
    for (unsigned int i = 0; i < num_landmarks; i++) {
        cv::KeyPoint kp;
        keypts.push_back(kp);
        scale_factors.emplace_back(0);
    }

    auto solver = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(bearings, keypts, landmarks, scale_factors));
    solver->find_via_ransac(30);
    EXPECT_TRUE(solver->solution_is_valid());

    const auto estimated_pose = solver->get_best_cam_pose();

    const auto rot = estimated_pose.block<3, 3>(0, 0);
    const auto trans = estimated_pose.block<3, 1>(0, 3);

    const auto rot_err = angle_of_rot(rot_gt * rot.transpose());
    const auto trans_err = (trans_gt - trans).norm();

    EXPECT_LT(rot_err, 1e-4);
    EXPECT_LT(trans_err, 1e-4);
}

TEST(pnp_solver, with_noise) {
    // Create three landmarks
    const unsigned int num_landmarks = 100;
    const auto landmarks = create_random_landmarks_in_space(num_landmarks, 100);

    // Create single-view pose
    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    // Create bearing vectors from pose and landmarks
    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, landmarks, bearings, 0.001);

    // Make dummy keypoints
    std::vector<cv::KeyPoint> keypts;
    std::vector<float> scale_factors;
    for (unsigned int i = 0; i < num_landmarks; i++) {
        cv::KeyPoint kp;
        keypts.push_back(kp);
        scale_factors.emplace_back(0);
    }

    auto solver = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(bearings, keypts, landmarks, scale_factors));
    solver->find_via_ransac(30);
    EXPECT_TRUE(solver->solution_is_valid());

    const auto estimated_pose = solver->get_best_cam_pose();

    const auto rot = estimated_pose.block<3, 3>(0, 0);
    const auto trans = estimated_pose.block<3, 1>(0, 3);

    const auto rot_err = angle_of_rot(rot_gt * rot.transpose());
    const auto trans_err = (trans_gt - trans).norm();

    EXPECT_LT(rot_err, 1e-2);
    EXPECT_LT(trans_err, 1);
}
