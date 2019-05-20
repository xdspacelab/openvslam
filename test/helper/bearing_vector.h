#ifndef OPENVSLAM_TEST_HELPER_BEARING_VECTOR_H
#define OPENVSLAM_TEST_HELPER_BEARING_VECTOR_H

#include "helper/landmark.h"

#include "openvslam/type.h"

using namespace openvslam;

void create_bearing_vectors(const Mat33_t& rot_1, const Vec3_t& trans_1, const Mat33_t& rot_2, const Vec3_t& trans_2,
                            const eigen_alloc_vector<Vec3_t>& landmarks, Mat33_t& E_21,
                            eigen_alloc_vector<Vec3_t>& bearings_1, eigen_alloc_vector<Vec3_t>& bearings_2,
                            double noise_stddev = 0.0);

#endif // OPENVSLAM_TEST_HELPER_BEARING_VECTOR_H
