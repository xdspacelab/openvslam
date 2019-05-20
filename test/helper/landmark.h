#ifndef OPENVSLAM_TEST_HELPER_LANDMARK_H
#define OPENVSLAM_TEST_HELPER_LANDMARK_H

#include "openvslam/type.h"

#include <random>

using namespace openvslam;

eigen_alloc_vector<Vec3_t> create_random_landmarks(const unsigned int num_landmarks = 2000, const float space_lim = 100.0);

#endif // OPENVSLAM_TEST_HELPER_LANDMARK_H
