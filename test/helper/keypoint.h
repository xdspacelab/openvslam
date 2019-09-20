#ifndef OPENVSLAM_TEST_HELPER_KEYPOINT_H
#define OPENVSLAM_TEST_HELPER_KEYPOINT_H

#include "openvslam/type.h"

using namespace openvslam;

void create_keypoints(const Mat33_t& rot_1, const Vec3_t& trans_1, const Mat33_t& cam_matrix_1, const Mat33_t& rot_2, const Vec3_t& trans_2, const Mat33_t& cam_matrix_2,
                      const eigen_alloc_vector<Vec3_t>& landmarks, std::vector<cv::Point2f>& keypoints_1, std::vector<cv::Point2f>& keypoints_2, double noise_stddev = 0.0);

void create_keypoints(const Mat33_t& rot_1, const Vec3_t& trans_1, const Mat33_t& cam_matrix_1, const Mat33_t& rot_2, const Vec3_t& trans_2, const Mat33_t& cam_matrix_2,
                      const eigen_alloc_vector<Vec3_t>& landmarks, std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2, double noise_stddev = 0.0);

#endif // OPENVSLAM_TEST_HELPER_KEYPOINT_H
