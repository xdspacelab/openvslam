#include "helper/landmark.h"
#include "helper/bearing_vector.h"

#include "openvslam/type.h"
#include "openvslam/util/converter.h"
#include "openvslam/solve/pnp_solver.h"

#include <memory>

#include <opencv2/core/types.hpp>

#include <gtest/gtest.h>

using namespace openvslam;

TEST(pnp_solver, without_ransac) {
    // Create four landmarks which needed the least number for solve problem
    const unsigned int num_landmarks = 10;
    eigen_alloc_vector<Vec3_t> landmarks;
    landmarks.emplace_back(Vec3_t{-19.283677, -18.130606, 82.329830});
    landmarks.emplace_back(Vec3_t{-88.230105, 61.669552, -52.896303});
    landmarks.emplace_back(Vec3_t{-46.048140, 47.097662, -92.047191});
    landmarks.emplace_back(Vec3_t{-91.468185, -56.584450, -35.762650});
    landmarks.emplace_back(Vec3_t{-82.214075, 11.124351, -0.022995});
    landmarks.emplace_back(Vec3_t{-88.117582, 84.359816, -55.239983});
    landmarks.emplace_back(Vec3_t{26.433690, -95.957955, -81.737696});
    landmarks.emplace_back(Vec3_t{43.528656, -0.451698, 84.015400});
    landmarks.emplace_back(Vec3_t{94.330351, 56.546052, 94.311132});
    landmarks.emplace_back(Vec3_t{6.822799, -5.799954, -88.622470});

    // Create single-view pose
    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    // Create bearing vectors from pose and landmarks
    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, landmarks, bearings);

    // keypts and scale_factor are required of solver
    // In this test, octave is 0 and scale factor is 1 for each keypoint
    std::vector<cv::KeyPoint> keypts;
    const std::vector<float> scale_factor{1};
    for (unsigned int i = 0; i < num_landmarks; i++) {
        keypts.emplace_back(cv::KeyPoint{});
    }

    // Compute the camera pose by pnp_solver
    auto solver = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(bearings, keypts, landmarks, scale_factor, 0));
    solver->find_via_ransac(30);

    EXPECT_TRUE(solver->solution_is_valid());

    // Compute error of estimated pose
    const auto estimated_pose = solver->get_best_cam_pose();

    const auto rot = estimated_pose.block<3, 3>(0, 0);
    const auto trans = estimated_pose.block<3, 1>(0, 3);

    const auto rot_err = util::converter::to_angle_axis(rot_gt * rot.transpose()).norm();
    const auto trans_err = (trans_gt - trans).norm();

    EXPECT_LT(rot_err, 1e-2);
    EXPECT_LT(trans_err, 1);
}

TEST(pnp_solver, without_noise) {
    // Create landmarks
    const unsigned int num_landmarks = 100;
    const auto landmarks = create_random_landmarks_in_space(num_landmarks, 100);

    // Create single-view pose
    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    // Create bearing vectors from pose and landmarks
    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, landmarks, bearings);

    // keypts and scale_factor are required of solver
    // In this test, octave is 0 and scale factor is 1 for each keypoint
    std::vector<cv::KeyPoint> keypts;
    const std::vector<float> scale_factor{1};
    for (unsigned int i = 0; i < num_landmarks; i++) {
        keypts.emplace_back(cv::KeyPoint{});
    }

    // Compute the camera pose by pnp_solver
    auto solver = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(bearings, keypts, landmarks, scale_factor));
    solver->find_via_ransac(30);
    EXPECT_TRUE(solver->solution_is_valid());

    const auto estimated_pose = solver->get_best_cam_pose();

    const auto rot = estimated_pose.block<3, 3>(0, 0);
    const auto trans = estimated_pose.block<3, 1>(0, 3);

    const auto rot_err = util::converter::to_angle_axis(rot_gt * rot.transpose()).norm();
    const auto trans_err = (trans_gt - trans).norm();

    EXPECT_LT(rot_err, 1e-4);
    EXPECT_LT(trans_err, 1e-4);
}

TEST(pnp_solver, with_noise) {
    // Create landmarks
    const unsigned int num_landmarks = 100;
    const auto landmarks = create_random_landmarks_in_space(num_landmarks, 100);

    // Create single-view pose
    const Mat33_t rot_gt = util::converter::to_rot_mat(97.37 * M_PI / 180 * Vec3_t{9.0, -8.5, 1.1}.normalized());
    const Vec3_t trans_gt = Vec3_t(-67.5, 84.6, -68.0);

    // Create bearing vectors containing observation noise
    eigen_alloc_vector<Vec3_t> bearings;
    create_bearing_vectors(rot_gt, trans_gt, landmarks, bearings, 0.0004);

    // keypts and scale_factor are required of solver
    // In this test, octave is 0 and scale factor is 1 for each keypoint
    std::vector<cv::KeyPoint> keypts;
    const std::vector<float> scale_factor{1};
    for (unsigned int i = 0; i < num_landmarks; i++) {
        keypts.emplace_back(cv::KeyPoint{});
    }

    // Compute the camera pose by pnp_solver
    auto solver = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(bearings, keypts, landmarks, scale_factor));
    solver->find_via_ransac(30);
    EXPECT_TRUE(solver->solution_is_valid());

    const auto estimated_pose = solver->get_best_cam_pose();

    const auto rot = estimated_pose.block<3, 3>(0, 0);
    const auto trans = estimated_pose.block<3, 1>(0, 3);

    const auto rot_err = util::converter::to_angle_axis(rot_gt * rot.transpose()).norm();
    const auto trans_err = (trans_gt - trans).norm();

    EXPECT_LT(rot_err, 1e-2);
    EXPECT_LT(trans_err, 1);
}
