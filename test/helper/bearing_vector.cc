#include "helper/bearing_vector.h"

#include <random>

void create_bearing_vectors(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const eigen_alloc_vector<Vec3_t>& landmarks,
                            eigen_alloc_vector<Vec3_t>& bearings, const double noise_stddev) {
    const auto num_landmarks = landmarks.size();

    // convert a 3D point to a bearing vector
    bearings.resize(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        const Vec3_t& lm = landmarks.at(i);
        const Vec3_t lm_in_1 = rot_cw * lm + trans_cw;
        bearings.at(i) = lm_in_1.normalized();
    }

    if (noise_stddev == 0.0) {
        return;
    }

    // add Gaussian noise

    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::normal_distribution<> rand(0, noise_stddev);

    for (unsigned int i = 0; i < num_landmarks; ++i) {
        bearings.at(i) += Vec3_t{rand(mt), rand(mt), rand(mt)};
        bearings.at(i).normalized();
    }
}
