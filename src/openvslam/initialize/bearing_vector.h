#ifndef OPENVSLAM_INITIALIZE_BEARING_VECTOR_H
#define OPENVSLAM_INITIALIZE_BEARING_VECTOR_H

#include "openvslam/type.h"
#include "openvslam/initialize/base.h"

#include <opencv2/opencv.hpp>

namespace openvslam {

namespace data {
class frame;
} // namespace data

namespace initialize {

class bearing_vector final : public base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bearing_vector() = delete;

    explicit bearing_vector(const data::frame& ref_frm, const unsigned int max_num_iters = 100);

    ~bearing_vector() override;

    bool initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) override;

private:
    bool reconstruct_with_E(const Mat33_t& E_ref_to_cur, const std::vector<bool>& is_inlier_match,
                            Mat33_t& rot_ref_to_cur, Vec3_t& trans_ref_to_cur,
                            eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                            const float min_parallax_deg = 1.0, const unsigned int min_num_triangulated = 50);
};

} // namespace initialize
} // namespace openvslam

#endif // OPENVSLAM_INITIALIZE_BEARING_VECTOR_H
