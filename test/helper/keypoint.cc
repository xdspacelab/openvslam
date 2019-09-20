#include "helper/keypoint.h"

#include <random>

void create_keypoints(const Mat33_t& rot_1, const Vec3_t& trans_1, const Mat33_t& cam_matrix_1, const Mat33_t& rot_2, const Vec3_t& trans_2, const Mat33_t& cam_matrix_2,
                      const eigen_alloc_vector<Vec3_t>& landmarks, std::vector<cv::Point2f>& keypoints_1, std::vector<cv::Point2f>& keypoints_2, double noise_stddev) {
    const auto num_landmarks = landmarks.size();

    // convert a 3D point to a bearing vector

    keypoints_1.resize(num_landmarks);
    keypoints_2.resize(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        const Vec3_t& lm = landmarks.at(i);

        Vec3_t lm_in_1 = rot_1 * lm + trans_1;
        lm_in_1 /= lm_in_1(2);
        Vec3_t lm_in_2 = rot_2 * lm + trans_2;
        lm_in_2 /= lm_in_2(2);

        keypoints_1.at(i).x = cam_matrix_1(0, 0) * lm_in_1(0) + cam_matrix_1(0, 2);
        keypoints_1.at(i).y = cam_matrix_1(1, 1) * lm_in_1(1) + cam_matrix_1(1, 2);
        keypoints_2.at(i).x = cam_matrix_2(0, 0) * lm_in_2(0) + cam_matrix_2(0, 2);
        keypoints_2.at(i).y = cam_matrix_2(1, 1) * lm_in_2(1) + cam_matrix_2(1, 2);
    }

    if (noise_stddev == 0.0) {
        return;
    }

    // add Gaussian noise

    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::normal_distribution<> rand(0, noise_stddev);

    for (unsigned int i = 0; i < num_landmarks; ++i) {
        keypoints_1.at(i) += cv::Point2f(rand(mt), rand(mt));
        keypoints_2.at(i) += cv::Point2f(rand(mt), rand(mt));
    }
}

void create_keypoints(const Mat33_t& rot_1, const Vec3_t& trans_1, const Mat33_t& cam_matrix_1, const Mat33_t& rot_2, const Vec3_t& trans_2, const Mat33_t& cam_matrix_2,
                      const eigen_alloc_vector<Vec3_t>& landmarks, std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2, double noise_stddev) {
    const auto num_landmarks = landmarks.size();

    // convert a 3D point to a bearing vector

    keypoints_1.resize(num_landmarks);
    keypoints_2.resize(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        const Vec3_t& lm = landmarks.at(i);

        Vec3_t lm_in_1 = rot_1 * lm + trans_1;
        lm_in_1 /= lm_in_1(2);
        Vec3_t lm_in_2 = rot_2 * lm + trans_2;
        lm_in_2 /= lm_in_2(2);

        keypoints_1.at(i).pt.x = cam_matrix_1(0, 0) * lm_in_1(0) + cam_matrix_1(0, 2);
        keypoints_1.at(i).pt.y = cam_matrix_1(1, 1) * lm_in_1(1) + cam_matrix_1(1, 2);
        keypoints_2.at(i).pt.x = cam_matrix_2(0, 0) * lm_in_2(0) + cam_matrix_2(0, 2);
        keypoints_2.at(i).pt.y = cam_matrix_2(1, 1) * lm_in_2(1) + cam_matrix_2(1, 2);
    }

    if (noise_stddev == 0.0) {
        return;
    }

    // add Gaussian noise

    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::normal_distribution<> rand(0, noise_stddev);

    for (unsigned int i = 0; i < num_landmarks; ++i) {
        keypoints_1.at(i).pt += cv::Point2f(rand(mt), rand(mt));
        keypoints_2.at(i).pt += cv::Point2f(rand(mt), rand(mt));
    }
}
