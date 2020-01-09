#ifndef OPENVSLAM_CAMERA_FISHEYE_H
#define OPENVSLAM_CAMERA_FISHEYE_H

#include "openvslam/camera/base.h"

#include <opencv2/calib3d.hpp>

namespace openvslam {
namespace camera {

class fisheye final : public base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    fisheye(const std::string& name, const setup_type_t& setup_type, const color_order_t& color_order,
            const unsigned int cols, const unsigned int rows, const double fps,
            const double fx, const double fy, const double cx, const double cy,
            const double k1, const double k2, const double k3, const double k4,
            const double focal_x_baseline = 0.0);

    fisheye(const YAML::Node& yaml_node);

    ~fisheye() override;

    void show_parameters() const override final;

    image_bounds compute_image_bounds() const override final;

    cv::KeyPoint undistort_keypoint(const cv::KeyPoint& dist_keypt) const override final {
        // fill cv::Mat with distorted keypoints
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = dist_keypt.pt.x;
        mat.at<float>(0, 1) = dist_keypt.pt.y;

        // undistort
        mat = mat.reshape(2);
        cv::fisheye::undistortPoints(mat, mat, cv_cam_matrix_, cv_dist_params_, cv::Mat(), cv_cam_matrix_);
        mat = mat.reshape(1);

        // convert to cv::Mat
        cv::KeyPoint undist_keypt;
        undist_keypt.pt.x = mat.at<float>(0, 0);
        undist_keypt.pt.y = mat.at<float>(0, 1);
        undist_keypt.angle = dist_keypt.angle;
        undist_keypt.size = dist_keypt.size;
        undist_keypt.octave = dist_keypt.octave;

        return undist_keypt;
    }

    void undistort_keypoints(const std::vector<cv::KeyPoint>& dist_keypt, std::vector<cv::KeyPoint>& undist_keypt) const override final;

    Vec3_t convert_keypoint_to_bearing(const cv::KeyPoint& undist_keypt) const override final {
        const auto x_normalized = (undist_keypt.pt.x - cx_) / fx_;
        const auto y_normalized = (undist_keypt.pt.y - cy_) / fy_;
        const auto l2_norm = std::sqrt(x_normalized * x_normalized + y_normalized * y_normalized + 1.0);
        return Vec3_t{x_normalized / l2_norm, y_normalized / l2_norm, 1.0 / l2_norm};
    }

    void convert_keypoints_to_bearings(const std::vector<cv::KeyPoint>& undist_keypt, eigen_alloc_vector<Vec3_t>& bearings) const override final;

    cv::KeyPoint convert_bearing_to_keypoint(const Vec3_t& bearing) const override final {
        const auto x_normalized = bearing(0) / bearing(2);
        const auto y_normalized = bearing(1) / bearing(2);

        cv::KeyPoint undist_keypt;
        undist_keypt.pt.x = fx_ * x_normalized + cx_;
        undist_keypt.pt.y = fy_ * y_normalized + cy_;

        return undist_keypt;
    }

    void convert_bearings_to_keypoints(const eigen_alloc_vector<Vec3_t>& bearings, std::vector<cv::KeyPoint>& undist_keypt) const override final;

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
    const double k1_;
    const double k2_;
    const double k3_;
    const double k4_;

    //! camera matrix in OpenCV format
    cv::Mat cv_cam_matrix_;
    //! camera matrix in Eigen format
    Mat33_t eigen_cam_matrix_;
    //! distortion params in OpenCV format
    cv::Mat cv_dist_params_;
    //! distortion params in Eigen format
    Vec4_t eigen_dist_params_;
};

} // namespace camera
} // namespace openvslam

#endif // OPENVSLAM_CAMERA_FISHEYE_H
