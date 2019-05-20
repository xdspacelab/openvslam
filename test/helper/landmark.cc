#include "helper/landmark.h"

eigen_alloc_vector<Vec3_t> create_random_landmarks(const unsigned int num_landmarks, const float space_lim) {
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
