#ifndef OPENVSLAM_INITIALIZE_PERSPECTIVE_H
#define OPENVSLAM_INITIALIZE_PERSPECTIVE_H

#include "openvslam/type.h"
#include "openvslam/camera/base.h"
#include "openvslam/initialize/base.h"

#include <opencv2/opencv.hpp>

namespace openvslam {

namespace data {
class frame;
} // namespace data

namespace initialize {

class perspective final : public base {
public:
    perspective() = delete;

    explicit perspective(const data::frame& ref_frm, const float sigma = 1.0, const unsigned int max_num_iters = 200);

    ~perspective() override;

    bool initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) override;

private:
    bool reconstruct_with_H(const Mat33_t& H_ref_to_cur, const std::vector<bool>& is_inlier_match,
                            Mat33_t& rot_ref_to_cur, Vec3_t& trans_ref_to_cur,
                            eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                            const float min_parallax_deg = 1.0, const unsigned int min_num_triangulated = 50);

    bool reconstruct_with_F(const Mat33_t& F_ref_to_cur, const std::vector<bool>& is_inlier_match,
                            Mat33_t& rot_ref_to_cur, Vec3_t& trans_ref_to_cur,
                            eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                            const float min_parallax_deg = 1.0, const unsigned int min_num_triangulated = 50);

    unsigned int check_pose(const Mat33_t& rot_ref_to_cur, const Vec3_t& trans_ref_to_cur,
                            const float reproj_err_thr_sq, const std::vector<bool>& is_inlier_match,
                            eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                            float& parallax);

    //! reprojection error threshold in pixel and pixel^2
    const float sigma_, sigma_sq_;

    // reference frame information
    //! camera model of reference frame
    Mat33_t ref_cam_matrix_;
    //! undistorted keypoints of reference frame
    std::vector<cv::KeyPoint> ref_undist_keypts_;

    // current frame information
    //! camera matrix of current frame
    Mat33_t cur_cam_matrix_;
    //! undistorted keypoints of current frame
    std::vector<cv::KeyPoint> cur_undist_keypts_;
};

} // namespace initialize
} // namespace openvslam

#endif // OPENVSLAM_INITIALIZE_PERSPECTIVE_H
