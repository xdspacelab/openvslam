#ifndef OPENVSLAM_TEST_HELPER_BEARING_VECTOR_H
#define OPENVSLAM_TEST_HELPER_BEARING_VECTOR_H

#include "openvslam/type.h"

using namespace openvslam;

void create_bearing_vectors(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const eigen_alloc_vector<Vec3_t>& landmarks,
                            eigen_alloc_vector<Vec3_t>& bearings, const double noise_stddev = 0.0);

#endif // OPENVSLAM_TEST_HELPER_BEARING_VECTOR_H
