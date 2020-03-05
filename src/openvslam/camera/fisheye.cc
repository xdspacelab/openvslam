#include "openvslam/camera/fisheye.h"

#include <iostream>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace openvslam {
namespace camera {

fisheye::fisheye(const std::string& name, const setup_type_t& setup_type, const color_order_t& color_order,
                 const unsigned int cols, const unsigned int rows, const double fps,
                 const double fx, const double fy, const double cx, const double cy,
                 const double k1, const double k2, const double k3, const double k4,
                 const double focal_x_baseline)
    : base(name, setup_type, model_type_t::Fisheye, color_order, cols, rows, fps, focal_x_baseline, focal_x_baseline / fx),
      fx_(fx), fy_(fy), cx_(cx), cy_(cy), fx_inv_(1.0 / fx), fy_inv_(1.0 / fy),
      k1_(k1), k2_(k2), k3_(k3), k4_(k4) {
    spdlog::debug("CONSTRUCT: camera::fisheye");

    cv_cam_matrix_ = (cv::Mat_<float>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
    cv_dist_params_ = (cv::Mat_<float>(4, 1) << k1_, k2_, k3_, k4_);

    eigen_cam_matrix_ << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
    eigen_dist_params_ << k1_, k2_, k3_, k4_;

    img_bounds_ = compute_image_bounds();

    inv_cell_width_ = static_cast<double>(num_grid_cols_) / (img_bounds_.max_x_ - img_bounds_.min_x_);
    inv_cell_height_ = static_cast<double>(num_grid_rows_) / (img_bounds_.max_y_ - img_bounds_.min_y_);
}

fisheye::fisheye(const YAML::Node& yaml_node)
    : fisheye(yaml_node["Camera.name"].as<std::string>(),
              load_setup_type(yaml_node),
              load_color_order(yaml_node),
              yaml_node["Camera.cols"].as<unsigned int>(),
              yaml_node["Camera.rows"].as<unsigned int>(),
              yaml_node["Camera.fps"].as<double>(),
              yaml_node["Camera.fx"].as<double>(),
              yaml_node["Camera.fy"].as<double>(),
              yaml_node["Camera.cx"].as<double>(),
              yaml_node["Camera.cy"].as<double>(),
              yaml_node["Camera.k1"].as<double>(),
              yaml_node["Camera.k2"].as<double>(),
              yaml_node["Camera.k3"].as<double>(),
              yaml_node["Camera.k4"].as<double>(),
              yaml_node["Camera.focal_x_baseline"].as<double>(0.0)) {}

fisheye::~fisheye() {
    spdlog::debug("DESTRUCT: camera::fisheye");
}

void fisheye::show_parameters() const {
    show_common_parameters();
    std::cout << "  - fx: " << fx_ << std::endl;
    std::cout << "  - fy: " << fy_ << std::endl;
    std::cout << "  - cx: " << cx_ << std::endl;
    std::cout << "  - cy: " << cy_ << std::endl;
    std::cout << "  - k1: " << k1_ << std::endl;
    std::cout << "  - k2: " << k2_ << std::endl;
    std::cout << "  - k3: " << k3_ << std::endl;
    std::cout << "  - k4: " << k4_ << std::endl;
    std::cout << "  - min x: " << img_bounds_.min_x_ << std::endl;
    std::cout << "  - max x: " << img_bounds_.max_x_ << std::endl;
    std::cout << "  - min y: " << img_bounds_.min_y_ << std::endl;
    std::cout << "  - max y: " << img_bounds_.max_y_ << std::endl;
}

image_bounds fisheye::compute_image_bounds() const {
    spdlog::debug("compute image bounds");

    if (k1_ == 0 && k2_ == 0 && k3_ == 0 && k4_ == 0) {
        // any distortion does not exist

        return image_bounds{0.0, cols_, 0.0, rows_};
    }
    else {
        // distortion exists

        // fix for issue #83
        // check if fov is super wide (four corners are out of view) based on upper-left corner
        const double pwx = (0.0 - cx_) / fx_;
        const double pwy = (0.0 - cy_) / fy_;
        const double theta_d = sqrt(pwx * pwx + pwy * pwy);

        if (theta_d > M_PI / 2) {
            // fov is super wide (four corners are out of view)

            // corner coordinates: (x, y) = (col, row)
            const std::vector<cv::KeyPoint> corners{cv::KeyPoint(cx_, 0.0, 1.0),    // top: min_y
                                                    cv::KeyPoint(cols_, cy_, 1.0),  // right: max_x
                                                    cv::KeyPoint(0.0, cy_, 1.0),    // left: min_x
                                                    cv::KeyPoint(cx_, rows_, 1.0)}; // down: max_y

            std::vector<cv::KeyPoint> undist_corners;
            undistort_keypoints(corners, undist_corners);

            // 1. limit image_bounds by incident angle
            //    when deg_thr = 5, only incident angle < 85 deg is accepted
            // 2. deal with over 180 deg fov
            //    some points over 180 deg are projected on the opposite side (plus or minus are different)
            constexpr float deg_thr = 5.0;
            const float dist_thr_x = fx_ / std::tan(deg_thr * M_PI / 180.0);
            const float dist_thr_y = fy_ / std::tan(deg_thr * M_PI / 180.0);
            const float min_x_thr = -dist_thr_x + cx_;
            const float max_x_thr = dist_thr_x + cx_;
            const float min_y_thr = -dist_thr_y + cy_;
            const float max_y_thr = dist_thr_y + cy_;
            const float undist_min_x = undist_corners.at(2).pt.x;
            const float undist_max_x = undist_corners.at(1).pt.x;
            const float undist_min_y = undist_corners.at(0).pt.y;
            const float undist_max_y = undist_corners.at(3).pt.y;
            return image_bounds{(undist_min_x < min_x_thr || undist_min_x > cx_) ? min_x_thr : undist_min_x,
                                (undist_max_x > max_x_thr || undist_max_x < cx_) ? max_x_thr : undist_max_x,
                                (undist_min_y < min_y_thr || undist_min_y > cy_) ? min_y_thr : undist_min_y,
                                (undist_max_y > max_y_thr || undist_max_y < cy_) ? max_y_thr : undist_max_y};
        }
        else {
            // fov is normal (four corners are inside of view)

            // corner coordinates: (x, y) = (col, row)
            const std::vector<cv::KeyPoint> corners{cv::KeyPoint(0.0, 0.0, 1.0),      // left top
                                                    cv::KeyPoint(cols_, 0.0, 1.0),    // right top
                                                    cv::KeyPoint(0.0, rows_, 1.0),    // left bottom
                                                    cv::KeyPoint(cols_, rows_, 1.0)}; // right bottom

            std::vector<cv::KeyPoint> undist_corners;
            undistort_keypoints(corners, undist_corners);

            return image_bounds{std::min(undist_corners.at(0).pt.x, undist_corners.at(2).pt.x),
                                std::max(undist_corners.at(1).pt.x, undist_corners.at(3).pt.x),
                                std::min(undist_corners.at(0).pt.y, undist_corners.at(1).pt.y),
                                std::max(undist_corners.at(2).pt.y, undist_corners.at(3).pt.y)};
        }
    }
}

void fisheye::undistort_keypoints(const std::vector<cv::KeyPoint>& dist_keypt, std::vector<cv::KeyPoint>& undist_keypt) const {
    // cv::fisheye::undistortPoints does not accept an empty input
    if (dist_keypt.empty()) {
        undist_keypt.clear();
        return;
    }

    // fill cv::Mat with distorted keypoints
    cv::Mat mat(dist_keypt.size(), 2, CV_32F);
    for (unsigned long idx = 0; idx < dist_keypt.size(); ++idx) {
        mat.at<float>(idx, 0) = dist_keypt.at(idx).pt.x;
        mat.at<float>(idx, 1) = dist_keypt.at(idx).pt.y;
    }

    // undistort
    mat = mat.reshape(2);
    cv::fisheye::undistortPoints(mat, mat, cv_cam_matrix_, cv_dist_params_, cv::Mat(), cv_cam_matrix_);
    mat = mat.reshape(1);

    // convert to cv::Mat
    undist_keypt.resize(dist_keypt.size());
    for (unsigned long idx = 0; idx < undist_keypt.size(); ++idx) {
        undist_keypt.at(idx).pt.x = mat.at<float>(idx, 0);
        undist_keypt.at(idx).pt.y = mat.at<float>(idx, 1);
        undist_keypt.at(idx).angle = dist_keypt.at(idx).angle;
        undist_keypt.at(idx).size = dist_keypt.at(idx).size;
        undist_keypt.at(idx).octave = dist_keypt.at(idx).octave;
    }
}

void fisheye::convert_keypoints_to_bearings(const std::vector<cv::KeyPoint>& undist_keypt, eigen_alloc_vector<Vec3_t>& bearings) const {
    bearings.resize(undist_keypt.size());
    for (unsigned long idx = 0; idx < undist_keypt.size(); ++idx) {
        const auto x_normalized = (undist_keypt.at(idx).pt.x - cx_) / fx_;
        const auto y_normalized = (undist_keypt.at(idx).pt.y - cy_) / fy_;
        const auto l2_norm = std::sqrt(x_normalized * x_normalized + y_normalized * y_normalized + 1.0);
        bearings.at(idx) = Vec3_t{x_normalized / l2_norm, y_normalized / l2_norm, 1.0 / l2_norm};
    }
}

void fisheye::convert_bearings_to_keypoints(const eigen_alloc_vector<Vec3_t>& bearings, std::vector<cv::KeyPoint>& undist_keypt) const {
    undist_keypt.resize(bearings.size());
    for (unsigned long idx = 0; idx < bearings.size(); ++idx) {
        const auto x_normalized = bearings.at(idx)(0) / bearings.at(idx)(2);
        const auto y_normalized = bearings.at(idx)(1) / bearings.at(idx)(2);

        undist_keypt.at(idx).pt.x = fx_ * x_normalized + cx_;
        undist_keypt.at(idx).pt.y = fy_ * y_normalized + cy_;
    }
}

bool fisheye::reproject_to_image(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec2_t& reproj, float& x_right) const {
    // convert to camera-coordinates
    const Vec3_t pos_c = rot_cw * pos_w + trans_cw;

    // check if the point is visible
    if (pos_c(2) <= 0.0) {
        return false;
    }

    // reproject onto the image
    const auto z_inv = 1.0 / pos_c(2);
    reproj(0) = fx_ * pos_c(0) * z_inv + cx_;
    reproj(1) = fy_ * pos_c(1) * z_inv + cy_;
    x_right = reproj(0) - focal_x_baseline_ * z_inv;

    // check if the point is visible
    return (img_bounds_.min_x_ < reproj(0) && reproj(0) < img_bounds_.max_x_
            && img_bounds_.min_y_ < reproj(1) && reproj(1) < img_bounds_.max_y_);
}

bool fisheye::reproject_to_bearing(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec3_t& reproj) const {
    // convert to camera-coordinates
    reproj = rot_cw * pos_w + trans_cw;

    // check if the point is visible
    if (reproj(2) <= 0.0) {
        return false;
    }

    // reproject onto the image
    const auto z_inv = 1.0 / reproj(2);
    const auto x = fx_ * reproj(0) * z_inv + cx_;
    const auto y = fy_ * reproj(1) * z_inv + cy_;

    // convert to a bearing
    reproj.normalize();

    // check if the point is visible
    return (img_bounds_.min_x_ < x && x < img_bounds_.max_x_
            && img_bounds_.min_y_ < y && y < img_bounds_.max_y_);
}

nlohmann::json fisheye::to_json() const {
    return {{"model_type", get_model_type_string()},
            {"setup_type", get_setup_type_string()},
            {"color_order", get_color_order_string()},
            {"cols", cols_},
            {"rows", rows_},
            {"fps", fps_},
            {"focal_x_baseline", focal_x_baseline_},
            {"num_grid_cols", num_grid_cols_},
            {"num_grid_rows", num_grid_rows_},
            {"fx", fx_},
            {"fy", fy_},
            {"cx", cx_},
            {"cy", cy_},
            {"k1", k1_},
            {"k2", k2_},
            {"k3", k3_},
            {"k4", k4_}};
}

} // namespace camera
} // namespace openvslam
