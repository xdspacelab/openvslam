#include "openvslam/feature/orb_params.h"

#include <cmath>

#include <gtest/gtest.h>

using namespace openvslam;

TEST(orb_params, load_yaml_without_rectangle_mask) {
    const std::string yaml =
        "Feature.max_num_keypoints: 3000\n"
        "Feature.scale_factor: 1.3\n"
        "Feature.num_levels: 12\n"
        "Feature.ini_fast_threshold: 25\n"
        "Feature.min_fast_threshold: 9\n";

    const auto yaml_node = YAML::Load(yaml);
    const auto params = feature::orb_params(yaml_node);

    EXPECT_EQ(params.max_num_keypts_, 3000);
    EXPECT_FLOAT_EQ(params.scale_factor_, 1.3);
    EXPECT_EQ(params.num_levels_, 12);
    EXPECT_EQ(params.ini_fast_thr_, 25);
    EXPECT_EQ(params.min_fast_thr, 9);
    EXPECT_EQ(params.mask_rects_.size(), 0);
}

TEST(orb_params, load_yaml_with_rectangle_mask) {
    const std::string yaml =
        "Camera.cols: 640\n"
        "Camera.rows: 480\n"
        "Feature.max_num_keypoints: 3000\n"
        "Feature.scale_factor: 1.3\n"
        "Feature.num_levels: 12\n"
        "Feature.ini_fast_threshold: 25\n"
        "Feature.min_fast_threshold: 9\n"
        "Feature.mask_rectangles:\n"
        "- [0.2, 0.5, 0.3, 0.8]\n"
        "- [0.28, 0.59, 0.1, 0.2]\n";

    const auto yaml_node = YAML::Load(yaml);
    const auto params = feature::orb_params(yaml_node);

    EXPECT_EQ(params.max_num_keypts_, 3000);
    EXPECT_FLOAT_EQ(params.scale_factor_, 1.3);
    EXPECT_EQ(params.num_levels_, 12);
    EXPECT_EQ(params.ini_fast_thr_, 25);
    EXPECT_EQ(params.min_fast_thr, 9);
    EXPECT_EQ(params.mask_rects_.size(), 2);

    EXPECT_FLOAT_EQ(params.mask_rects_.at(0).at(0), 0.2);
    EXPECT_FLOAT_EQ(params.mask_rects_.at(0).at(1), 0.5);
    EXPECT_FLOAT_EQ(params.mask_rects_.at(0).at(2), 0.3);
    EXPECT_FLOAT_EQ(params.mask_rects_.at(0).at(3), 0.8);

    EXPECT_FLOAT_EQ(params.mask_rects_.at(1).at(0), 0.28);
    EXPECT_FLOAT_EQ(params.mask_rects_.at(1).at(1), 0.59);
    EXPECT_FLOAT_EQ(params.mask_rects_.at(1).at(2), 0.1);
    EXPECT_FLOAT_EQ(params.mask_rects_.at(1).at(3), 0.2);
}

TEST(orb_params, load_yaml_with_rectangle_mask_exception_1) {
    // when the size of vector in mask_rectangles is not four
    const std::string yaml =
        "Camera.cols: 640\n"
        "Camera.rows: 480\n"
        "Feature.max_num_keypoints: 3000\n"
        "Feature.scale_factor: 1.3\n"
        "Feature.num_levels: 12\n"
        "Feature.ini_fast_threshold: 25\n"
        "Feature.min_fast_threshold: 9\n"
        "Feature.mask_rectangles:\n"
        "- [0.2, 0.5, 0.3, 0.8]\n"
        "- [0.28, 0.59, 0.1]\n";

    const auto yaml_node = YAML::Load(yaml);
    EXPECT_THROW(const auto params = feature::orb_params(yaml_node), std::runtime_error);
}

TEST(orb_params, load_yaml_with_rectangle_mask_exception_2) {
    // when x_min equals x_max
    const std::string yaml =
        "Camera.cols: 640\n"
        "Camera.rows: 480\n"
        "Feature.max_num_keypoints: 3000\n"
        "Feature.scale_factor: 1.3\n"
        "Feature.num_levels: 12\n"
        "Feature.ini_fast_threshold: 25\n"
        "Feature.min_fast_threshold: 9\n"
        "Feature.mask_rectangles:\n"
        "- [0.2, 0.5, 0.3, 0.8]\n"
        "- [0.28, 0.28, 0.1, 0.2]\n";

    const auto yaml_node = YAML::Load(yaml);
    EXPECT_THROW(const auto params = feature::orb_params(yaml_node), std::runtime_error);
}

TEST(orb_params, load_yaml_with_rectangle_mask_exception_3) {
    // when x_min is greater than x_max
    const std::string yaml =
        "Camera.cols: 640\n"
        "Camera.rows: 480\n"
        "Feature.max_num_keypoints: 3000\n"
        "Feature.scale_factor: 1.3\n"
        "Feature.num_levels: 12\n"
        "Feature.ini_fast_threshold: 25\n"
        "Feature.min_fast_threshold: 9\n"
        "Feature.mask_rectangles:\n"
        "- [0.2, 0.5, 0.3, 0.8]\n"
        "- [0.25, 0.21, 0.1, 0.2]\n";

    const auto yaml_node = YAML::Load(yaml);
    EXPECT_THROW(const auto params = feature::orb_params(yaml_node), std::runtime_error);
}

TEST(orb_params, load_yaml_with_rectangle_mask_exception_4) {
    // when y_min equals y_max
    const std::string yaml =
        "Camera.cols: 640\n"
        "Camera.rows: 480\n"
        "Feature.max_num_keypoints: 3000\n"
        "Feature.scale_factor: 1.3\n"
        "Feature.num_levels: 12\n"
        "Feature.ini_fast_threshold: 25\n"
        "Feature.min_fast_threshold: 9\n"
        "Feature.mask_rectangles:\n"
        "- [0.2, 0.5, 0.3, 0.8]\n"
        "- [0.28, 0.59, 0.8, 0.8]\n";

    const auto yaml_node = YAML::Load(yaml);
    EXPECT_THROW(const auto params = feature::orb_params(yaml_node), std::runtime_error);
}

TEST(orb_params, load_yaml_with_rectangle_mask_exception_5) {
    // when y_min is greater than y_max
    const std::string yaml =
        "Camera.cols: 640\n"
        "Camera.rows: 480\n"
        "Feature.max_num_keypoints: 3000\n"
        "Feature.scale_factor: 1.3\n"
        "Feature.num_levels: 12\n"
        "Feature.ini_fast_threshold: 25\n"
        "Feature.min_fast_threshold: 9\n"
        "Feature.mask_rectangles:\n"
        "- [0.2, 0.5, 0.3, 0.8]\n"
        "- [0.28, 0.59, 0.7, 0.2]\n";

    const auto yaml_node = YAML::Load(yaml);
    EXPECT_THROW(const auto params = feature::orb_params(yaml_node), std::runtime_error);
}

TEST(orb_params, calc_scale_factors) {
    const unsigned int num_scale_levels = 10;
    const float scale_factor = 1.26;

    const auto scale_factors = feature::orb_params::calc_scale_factors(num_scale_levels, scale_factor);

    for (unsigned int level = 0; level < num_scale_levels; ++level) {
        EXPECT_FLOAT_EQ(scale_factors.at(level), std::pow(scale_factor, level));
    }
}

TEST(orb_params, calc_inv_scale_factors) {
    const unsigned int num_scale_levels = 10;
    const float scale_factor = 1.26;

    const auto inv_scale_factors = feature::orb_params::calc_inv_scale_factors(num_scale_levels, scale_factor);

    for (unsigned int level = 0; level < num_scale_levels; ++level) {
        EXPECT_FLOAT_EQ(inv_scale_factors.at(level), std::pow((1.0f / scale_factor), level));
    }
}

TEST(orb_params, calc_level_sigma_sq) {
    const unsigned int num_scale_levels = 10;
    const float scale_factor = 1.26;

    const auto level_sigma_sq = feature::orb_params::calc_level_sigma_sq(num_scale_levels, scale_factor);
    float scale_factor_at_level = 1.0;
    for (unsigned int level = 0; level < num_scale_levels; ++level) {
        EXPECT_FLOAT_EQ(level_sigma_sq.at(level), scale_factor_at_level * scale_factor_at_level);
        scale_factor_at_level = scale_factor * scale_factor_at_level;
    }
}

TEST(orb_params, calc_inv_level_sigma_sq) {
    const unsigned int num_scale_levels = 10;
    const float scale_factor = 1.26;

    const auto inv_level_sigma_sq = feature::orb_params::calc_inv_level_sigma_sq(num_scale_levels, scale_factor);
    float scale_factor_at_level = 1.0;
    for (unsigned int level = 0; level < num_scale_levels; ++level) {
        EXPECT_FLOAT_EQ(inv_level_sigma_sq.at(level), 1.0f / (scale_factor_at_level * scale_factor_at_level));
        scale_factor_at_level = scale_factor * scale_factor_at_level;
    }
}
