#include "helper/bearing_vector.h"

#include <Eigen/Core>
#include <Eigen/LU>

void create_bearing_vectors(const Mat33_t& rot_1, const Vec3_t& trans_1, const Mat33_t& rot_2, const Vec3_t& trans_2,
                            const eigen_alloc_vector<Vec3_t>& landmarks, Mat33_t& E_21,
                            eigen_alloc_vector<Vec3_t>& bearings_1, eigen_alloc_vector<Vec3_t>& bearings_2,
                            double noise_stddev) {
    const auto num_landmarks = landmarks.size();

    // E行列を求める

    const Mat33_t rot_21 = rot_2 * rot_1.inverse();
    const Vec3_t trans_21 = trans_2 - rot_21 * trans_1;

    Mat33_t skew_21;
    skew_21 << 0, -trans_21(2), trans_21(1),
               trans_21(2), 0, -trans_21(0),
               -trans_21(1), trans_21(0), 0;

    E_21 = skew_21 * rot_21;

    // bearing vectorsを求める

    bearings_1.resize(num_landmarks);
    bearings_2.resize(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        const Vec3_t& lm = landmarks.at(i);

        const Vec3_t lm_in_1 = rot_1 * lm + trans_1;
        const Vec3_t lm_in_2 = rot_2 * lm + trans_2;

        bearings_1.at(i) = lm_in_1.normalized();
        bearings_2.at(i) = lm_in_2.normalized();
    }

    if (noise_stddev == 0.0) {
        return;
    }

    // ノイズを追加

    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::normal_distribution<> rand(0, noise_stddev);

    for (unsigned int i = 0; i < num_landmarks; ++i) {
        bearings_1.at(i) += Vec3_t{rand(mt), rand(mt), rand(mt)};
        bearings_1.at(i).normalized();
        bearings_2.at(i) += Vec3_t{rand(mt), rand(mt), rand(mt)};
        bearings_2.at(i).normalized();
    }
}
