#include "openvslam/camera/base.h"

#include <iostream>

#include <spdlog/spdlog.h>

namespace openvslam {
namespace camera {

base::base(const std::string& name, const setup_type_t setup_type, const model_type_t model_type, const color_order_t color_order,
           const unsigned int cols, const unsigned int rows, const double fps,
           const double focal_x_baseline, const double true_baseline,
           const unsigned int num_grid_cols, const unsigned int num_grid_rows)
    : name_(name), setup_type_(setup_type), model_type_(model_type), color_order_(color_order),
      cols_(cols), rows_(rows), fps_(fps),
      focal_x_baseline_(focal_x_baseline), true_baseline_(true_baseline),
      num_grid_cols_(num_grid_cols), num_grid_rows_(num_grid_rows) {
    spdlog::debug("CONSTRUCT: camera::base");
}

base::~base() {
    spdlog::debug("DESTRUCT: camera::base");
}

setup_type_t base::load_setup_type(const YAML::Node& yaml_node) {
    const auto setup_type_str = yaml_node["Camera.setup"].as<std::string>();
    if (setup_type_str == "monocular") {
        return camera::setup_type_t::Monocular;
    }
    else if (setup_type_str == "stereo") {
        return camera::setup_type_t::Stereo;
    }
    else if (setup_type_str == "RGBD") {
        return camera::setup_type_t::RGBD;
    }

    throw std::runtime_error("Invalid setup type: " + setup_type_str);
}

setup_type_t base::load_setup_type(const std::string& setup_type_str) {
    const auto itr = std::find(setup_type_to_string.begin(), setup_type_to_string.end(), setup_type_str);
    if (itr == setup_type_to_string.end()) {
        throw std::runtime_error("Invalid setup type: " + setup_type_str);
    }
    return static_cast<setup_type_t>(std::distance(setup_type_to_string.begin(), itr));
}

model_type_t base::load_model_type(const YAML::Node& yaml_node) {
    const auto model_type_str = yaml_node["Camera.model"].as<std::string>();
    if (model_type_str == "perspective") {
        return camera::model_type_t::Perspective;
    }
    else if (model_type_str == "fisheye") {
        return camera::model_type_t::Fisheye;
    }
    else if (model_type_str == "equirectangular") {
        return camera::model_type_t::Equirectangular;
    }
    else if (model_type_str == "radial_division") {
        return camera::model_type_t::RadialDivision;
    }
    throw std::runtime_error("Invalid camera model: " + model_type_str);
}

model_type_t base::load_model_type(const std::string& model_type_str) {
    const auto itr = std::find(model_type_to_string.begin(), model_type_to_string.end(), model_type_str);
    if (itr == model_type_to_string.end()) {
        throw std::runtime_error("Invalid camera model: " + model_type_str);
    }
    return static_cast<model_type_t>(std::distance(model_type_to_string.begin(), itr));
}

color_order_t base::load_color_order(const YAML::Node& yaml_node) {
    if (!yaml_node["Camera.color_order"]) {
        return color_order_t::Gray;
    }

    const auto color_order_str = yaml_node["Camera.color_order"].as<std::string>();
    if (color_order_str == "Gray") {
        return color_order_t::Gray;
    }
    else if (color_order_str == "RGB" || color_order_str == "RGBA") {
        return color_order_t::RGB;
    }
    else if (color_order_str == "BGR" || color_order_str == "BGRA") {
        return color_order_t::BGR;
    }

    throw std::runtime_error("Invalid color order: " + color_order_str);
}

color_order_t base::load_color_order(const std::string& color_order_str) {
    const auto itr = std::find(color_order_to_string.begin(), color_order_to_string.end(), color_order_str);
    if (itr == color_order_to_string.end()) {
        throw std::runtime_error("Invalid color order: " + color_order_str);
    }
    return static_cast<color_order_t>(std::distance(color_order_to_string.begin(), itr));
}

void base::show_common_parameters() const {
    std::cout << "- name: " << name_ << std::endl;
    std::cout << "- setup: " << get_setup_type_string() << std::endl;
    std::cout << "- fps: " << fps_ << std::endl;
    std::cout << "- cols: " << cols_ << std::endl;
    std::cout << "- rows: " << rows_ << std::endl;
    std::cout << "- color: " << get_color_order_string() << std::endl;
    std::cout << "- model: " << get_model_type_string() << std::endl;
}

} // namespace camera
} // namespace openvslam
