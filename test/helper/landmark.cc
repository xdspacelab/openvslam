#include "helper/landmark.h"

eigen_alloc_vector<Vec3_t> create_random_landmarks_in_space(const unsigned int num_landmarks,
                                                            const float space_lim) {
    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::uniform_real_distribution<> rand(-space_lim, space_lim);

    eigen_alloc_vector<Vec3_t> landmarks(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        landmarks.at(i)(0) = rand(mt);
        landmarks.at(i)(1) = rand(mt);
        landmarks.at(i)(2) = rand(mt);
    }

    return landmarks;
}

eigen_alloc_vector<Vec3_t> create_random_landmarks_on_plane(const unsigned int num_landmarks,
                                                            const float space_lim, const Vec4_t& plane_coeffs) {
    std::random_device rand_dev;
    std::mt19937 mt(rand_dev());
    std::uniform_real_distribution<> rand(-space_lim, space_lim);

    const auto a = plane_coeffs(0);
    const auto b = plane_coeffs(1);
    const auto c = plane_coeffs(2);
    const auto d = plane_coeffs(3);

    eigen_alloc_vector<Vec3_t> landmarks(num_landmarks);
    for (unsigned int i = 0; i < num_landmarks; ++i) {
        const double x = rand(mt);
        const double y = rand(mt);
        const double z = -(d + a * x + b * y) / c;
        if (z < -space_lim || space_lim < z) {
            --i;
            continue;
        }
        landmarks.at(i)(0) = x;
        landmarks.at(i)(1) = y;
        landmarks.at(i)(2) = z;
    }

    return landmarks;
}
