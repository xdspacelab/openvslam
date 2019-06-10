#ifndef OPENVSLAM_INITIALIZE_BASE_H
#define OPENVSLAM_INITIALIZE_BASE_H

#include "openvslam/type.h"

#include <vector>

namespace openvslam {

namespace camera {
class base;
} // namespace camera

namespace data {
class frame;
} // namespace data

namespace initialize {

class base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    base() = delete;

    base(const data::frame& ref_frm, const unsigned int max_num_iters);

    virtual ~base() = default;

    virtual bool initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) = 0;

    Mat33_t get_rotation_ref_to_cur() const;

    Vec3_t get_translation_ref_to_cur() const;

    eigen_alloc_vector<Vec3_t> get_triangulated_pts() const;

    std::vector<bool> get_triangulated_flags() const;

protected:
    /**
     * Find the most plausible pose and set them to the member variables (outputs)
     */
    bool find_most_plausible_pose(const eigen_alloc_vector<Mat33_t>& init_rots, const eigen_alloc_vector<Vec3_t>& init_transes,
                                  const std::vector<bool>& is_inlier_match, const bool depth_is_positive);

    /**
     * Check the reconstructed camera poses via triangulation
     */
    unsigned int check_pose(const Mat33_t& rot_ref_to_cur, const Vec3_t& trans_ref_to_cur, const float reproj_err_thr_sq,
                            const std::vector<bool>& is_inlier_match, const bool depth_is_positive,
                            eigen_alloc_vector<Vec3_t>& triangulated_pts, std::vector<bool>& is_triangulated,
                            float& parallax_deg);

    //-----------------------------------------
    // reference frame information

    //! camera model of reference frame
    camera::base* const ref_camera_;
    //! undistorted keypoints of reference frame
    const std::vector<cv::KeyPoint> ref_undist_keypts_;
    //! bearing vectors of reference frame
    const eigen_alloc_vector<Vec3_t> ref_bearings_;

    //-----------------------------------------
    // current frame information

    //! camera matrix of current frame
    camera::base* cur_camera_;
    //! undistorted keypoints of current frame
    std::vector<cv::KeyPoint> cur_undist_keypts_;
    //! bearing vectors of current frame
    eigen_alloc_vector<Vec3_t> cur_bearings_;

    //-----------------------------------------
    // matching information

    //! matching between reference and current frames
    std::vector<std::pair<int, int>> ref_cur_matches_;

    //-----------------------------------------
    // parameters

    //! max number of iterations of RANSAC
    const unsigned int max_num_iters_;
    //! min number of triangulated pts
    const unsigned int min_num_triangulated_ = 50;
    //! min parallax
    const float min_parallax_deg_ = 1.0;

    //-----------------------------------------
    // outputs

    //! initial rotation from reference to current
    Mat33_t rot_ref_to_cur_ = Mat33_t::Identity();
    //! initial translation from reference to current
    Vec3_t trans_ref_to_cur_ = Vec3_t::Zero();
    //! triangulated pts, with respect to indices of reference frame
    eigen_alloc_vector<Vec3_t> triangulated_pts_;
    //! each indices of reference frame is successfully triangulated or not
    std::vector<bool> is_triangulated_;
};

} // namespace initialize
} // namespace openvslam

#endif // OPENVSLAM_INITIALIZE_BASE_H
