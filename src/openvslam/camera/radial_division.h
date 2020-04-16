#ifndef OPENVSLAM_CAMERA_RADIAL_DIVISION_H
#define OPENVSLAM_CAMERA_RADIAL_DIVISION_H

#include "openvslam/camera/base.h"

namespace openvslam {
namespace camera {

// This class implements the camera model presented in:
//
//   "Simultaneous linear estimation of multiple view geometry and lens
//   distortion" by Andrew Fitzgibbon, CVPR 2001.
//
// The model is easy to implement and fast to evaluate.
// It is well suited for wide angle lenses like used in action cameras
// implemented by Steffen Urban, March 2020 (urbste@googlemail.com)
class radial_division final : public base {
public:
    radial_division(const std::string& name, const setup_type_t& setup_type, const color_order_t& color_order,
                    const unsigned int cols, const unsigned int rows, const double fps,
                    const double fx, const double fy, const double cx, const double cy,
                    const double distortion, const double focal_x_baseline = 0.0);

    radial_division(const YAML::Node& yaml_node);

    ~radial_division() override;

    void show_parameters() const override final;

    image_bounds compute_image_bounds() const override final;

    inline cv::KeyPoint undistort_keypoint(const cv::KeyPoint& dist_keypt) const override final {
        // undistort
        const double pixel_x = (dist_keypt.pt.x - cx_) / fx_;
        const double pixel_y = (dist_keypt.pt.y - cy_) / fy_;
        const double radius_distorted_squared = pixel_x * pixel_x + pixel_y * pixel_y;
        const double undistortion = 1.0 + distortion_ * radius_distorted_squared;

        const double undistorted_pt_x = pixel_x / undistortion;
        const double undistorted_pt_y = pixel_y / undistortion;

        cv::KeyPoint undist_keypt;
        undist_keypt.pt.x = undistorted_pt_x * fx_ + cx_;
        undist_keypt.pt.y = undistorted_pt_y * fy_ + cy_;
        undist_keypt.angle = dist_keypt.pt.x;
        undist_keypt.size = dist_keypt.size;
        undist_keypt.octave = dist_keypt.octave;

        return undist_keypt;
    }

    void undistort_keypoints(const std::vector<cv::KeyPoint>& dist_keypts, std::vector<cv::KeyPoint>& undist_keypts) const override final;

    Vec3_t convert_keypoint_to_bearing(const cv::KeyPoint& undist_keypt) const override final {
        const auto x_normalized = (undist_keypt.pt.x - cx_) / fx_;
        const auto y_normalized = (undist_keypt.pt.y - cy_) / fy_;
        const auto l2_norm = std::sqrt(x_normalized * x_normalized + y_normalized * y_normalized + 1.0);
        return Vec3_t{x_normalized / l2_norm, y_normalized / l2_norm, 1.0 / l2_norm};
    }

    void convert_keypoints_to_bearings(const std::vector<cv::KeyPoint>& undist_keypts, eigen_alloc_vector<Vec3_t>& bearings) const override final;

    inline cv::KeyPoint convert_bearing_to_keypoint(const Vec3_t& bearing) const override final {
        const auto x_normalized = bearing(0) / bearing(2);
        const auto y_normalized = bearing(1) / bearing(2);

        cv::KeyPoint undist_keypt;
        undist_keypt.pt.x = fx_ * x_normalized + cx_;
        undist_keypt.pt.y = fy_ * y_normalized + cy_;

        return undist_keypt;
    }

    void convert_bearings_to_keypoints(const eigen_alloc_vector<Vec3_t>& bearings, std::vector<cv::KeyPoint>& undist_keypts) const override final;

    bool reproject_to_image(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec2_t& reproj, float& x_right) const override final;

    bool reproject_to_bearing(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec3_t& reproj) const override final;

    nlohmann::json to_json() const override final;

    //-------------------------
    // Parameters specific to this model

    //! pinhole params
    const double fx_;
    const double fy_;
    const double cx_;
    const double cy_;
    const double fx_inv_;
    const double fy_inv_;

    //! distortion params
    const double distortion_;

    //! camera matrix in OpenCV format
    cv::Mat cv_cam_matrix_;
    //! camera matrix in Eigen format
    Mat33_t eigen_cam_matrix_;
};

} // namespace camera
} // namespace openvslam

#endif // OPENVSLAM_CAMERA_RADIAL_DIVISION_H
