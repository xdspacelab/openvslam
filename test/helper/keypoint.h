#ifndef OPENVSLAM_TEST_HELPER_KEYPOINT_H
#define OPENVSLAM_TEST_HELPER_KEYPOINT_H

#include "openvslam/type.h"

using namespace openvslam;

void create_keypoints(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix, const eigen_alloc_vector<Vec3_t>& landmarks,
                      std::vector<cv::Point2f>& keypts, const double noise_stddev = 0.0);

void create_keypoints(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Mat33_t& cam_matrix, const eigen_alloc_vector<Vec3_t>& landmarks,
                      std::vector<cv::KeyPoint>& keypts, const double noise_stddev = 0.0);

#endif // OPENVSLAM_TEST_HELPER_KEYPOINT_H
