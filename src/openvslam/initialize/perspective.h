#ifndef OPENVSLAM_INITIALIZE_PERSPECTIVE_H
#define OPENVSLAM_INITIALIZE_PERSPECTIVE_H

#include "openvslam/type.h"
#include "openvslam/initialize/base.h"

#include <opencv2/opencv.hpp>

namespace openvslam {

namespace data {
class frame;
} // namespace data

namespace initialize {

class perspective final : public base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    perspective() = delete;

    explicit perspective(const data::frame& ref_frm, const unsigned int max_num_iters = 100);

    ~perspective() override;

    bool initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) override;

private:
    static Mat33_t get_camera_matrix(camera::base* camera);

    bool reconstruct_with_H(const Mat33_t& H_ref_to_cur, const std::vector<bool>& is_inlier_match,
                            Mat33_t& rot_ref_to_cur, Vec3_t& trans_ref_to_cur,
                            eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                            const float min_parallax_deg = 1.0, const unsigned int min_num_triangulated = 50);

    bool reconstruct_with_F(const Mat33_t& F_ref_to_cur, const std::vector<bool>& is_inlier_match,
                            Mat33_t& rot_ref_to_cur, Vec3_t& trans_ref_to_cur,
                            eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                            const float min_parallax_deg = 1.0, const unsigned int min_num_triangulated = 50);

    //! camera matrix of the reference frame
    const Mat33_t ref_cam_matrix_;
    //! camera matrix of the current frame
    Mat33_t cur_cam_matrix_;
};

} // namespace initialize
} // namespace openvslam

#endif // OPENVSLAM_INITIALIZE_PERSPECTIVE_H
