#include "openvslam/camera/equirectangular.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace openvslam {
namespace camera {

equirectangular::equirectangular(const std::string& name, const color_order_t& color_order,
                                 const unsigned int cols, const unsigned int rows, const double fps)
    : base(name, setup_type_t::Monocular, model_type_t::Equirectangular, color_order, cols, rows, fps, 0.0, 0.0) {
    spdlog::debug("CONSTRUCT: camera::equirectangular");

    img_bounds_ = compute_image_bounds();

    inv_cell_width_ = static_cast<double>(num_grid_cols_) / (img_bounds_.max_x_ - img_bounds_.min_x_);
    inv_cell_height_ = static_cast<double>(num_grid_rows_) / (img_bounds_.max_y_ - img_bounds_.min_y_);
}

equirectangular::equirectangular(const YAML::Node& yaml_node)
    : equirectangular(yaml_node["Camera.name"].as<std::string>(),
                      load_color_order(yaml_node),
                      yaml_node["Camera.cols"].as<unsigned int>(),
                      yaml_node["Camera.rows"].as<unsigned int>(),
                      yaml_node["Camera.fps"].as<double>()) {}

equirectangular::~equirectangular() {
    spdlog::debug("DESTRUCT: camera::equirectangular");
}

void equirectangular::show_parameters() const {
    show_common_parameters();
}

image_bounds equirectangular::compute_image_bounds() const {
    spdlog::debug("compute image bounds");

    return image_bounds{0.0, cols_, 0.0, rows_};
}

void equirectangular::undistort_keypoints(const std::vector<cv::KeyPoint>& dist_keypts, std::vector<cv::KeyPoint>& undist_keypts) const {
    undist_keypts = dist_keypts;
}

void equirectangular::convert_keypoints_to_bearings(const std::vector<cv::KeyPoint>& undist_keypts, eigen_alloc_vector<Vec3_t>& bearings) const {
    bearings.resize(undist_keypts.size());
    for (unsigned int idx = 0; idx < undist_keypts.size(); ++idx) {
        // convert to unit polar coordinates
        const double lon = (undist_keypts.at(idx).pt.x / cols_ - 0.5) * (2 * M_PI);
        const double lat = -(undist_keypts.at(idx).pt.y / rows_ - 0.5) * M_PI;
        // convert to equirectangular coordinates
        bearings.at(idx)(0) = std::cos(lat) * std::sin(lon);
        bearings.at(idx)(1) = -std::sin(lat);
        bearings.at(idx)(2) = std::cos(lat) * std::cos(lon);
    }
}

void equirectangular::convert_bearings_to_keypoints(const eigen_alloc_vector<Vec3_t>& bearings, std::vector<cv::KeyPoint>& undist_keypts) const {
    undist_keypts.resize(bearings.size());
    for (unsigned int idx = 0; idx < bearings.size(); ++idx) {
        // convert to unit polar coordinates
        const double lat = -std::asin(bearings.at(idx)[1]);
        const double lon = std::atan2(bearings.at(idx)[0], bearings.at(idx)[2]);
        // convert to pixel image coordinated
        undist_keypts.at(idx).pt.x = cols_ * (0.5 + lon / (2 * M_PI));
        undist_keypts.at(idx).pt.y = rows_ * (0.5 - lat / M_PI);
    }
}

bool equirectangular::reproject_to_image(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec2_t& reproj, float& x_right) const {
    // convert to camera-coordinates
    const Vec3_t bearing = (rot_cw * pos_w + trans_cw).normalized();

    // convert to unit polar coordinates
    const auto latitude = -std::asin(bearing(1));
    const auto longitude = std::atan2(bearing(0), bearing(2));

    // convert to pixel image coordinated
    reproj(0) = cols_ * (0.5 + longitude / (2.0 * M_PI));
    reproj(1) = rows_ * (0.5 - latitude / M_PI);
    x_right = 0.0;

    return true;
}

bool equirectangular::reproject_to_bearing(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec3_t& reproj) const {
    // convert to camera-coordinates
    reproj = (rot_cw * pos_w + trans_cw).normalized();

    return true;
}

nlohmann::json equirectangular::to_json() const {
    return {{"model_type", get_model_type_string()},
            {"setup_type", get_setup_type_string()},
            {"color_order", get_color_order_string()},
            {"cols", cols_},
            {"rows", rows_},
            {"fps", fps_},
            {"focal_x_baseline", focal_x_baseline_},
            {"num_grid_cols", num_grid_cols_},
            {"num_grid_rows", num_grid_rows_}};
}

} // namespace camera
} // namespace openvslam
