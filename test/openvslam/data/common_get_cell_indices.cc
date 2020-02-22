#include "openvslam/data/common.h"
#include "openvslam/camera/perspective.h"

#include <gtest/gtest.h>

using namespace openvslam;

camera::perspective create_perspective_camera(const unsigned int cols, const unsigned int rows,
                                              const float k1 = 0.0, const float k2 = 0.0) {
    using namespace camera;
    return perspective("perspective", setup_type_t::Monocular, color_order_t::RGB,
                       cols, rows, 30.0, static_cast<double>(rows), static_cast<double>(rows),
                       cols / 2.0, rows / 2.0, k1, k2, 0.0, 0.0, 0.0);
}

TEST(common, valid_cases_1) {
    // create an example perspective camera
    constexpr unsigned int cols = 2000;
    constexpr unsigned int rows = 1000;
    auto cam = create_perspective_camera(cols, rows, -0.1, 0.1);
    // create keypoints and those grid IDs
    std::vector<std::pair<cv::KeyPoint, std::pair<int, int>>> test_cases;
    constexpr float eps = 0.01;
    // - corners
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.min_x_, cam.img_bounds_.min_y_, 0.0},
                                           std::make_pair(0, 0))); // top left
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.max_x_ - eps, cam.img_bounds_.min_y_, 0.0},
                                           std::make_pair(cam.num_grid_cols_ - 1, 0))); // top right
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.min_x_, cam.img_bounds_.max_y_ - eps, 0.0},
                                           std::make_pair(0, cam.num_grid_rows_ - 1))); // bottop left
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.max_x_ - eps, cam.img_bounds_.max_y_ - eps, 0.0},
                                           std::make_pair(cam.num_grid_cols_ - 1, cam.num_grid_rows_ - 1))); // bottom right
    // - edges
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cols / 2.0, cam.img_bounds_.min_y_, 0.0},
                                           std::make_pair(cam.num_grid_cols_ / 2 - 1, 0))); // top center
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cols / 2.0, cam.img_bounds_.max_y_ - eps, 0.0},
                                           std::make_pair(cam.num_grid_cols_ / 2 - 1, cam.num_grid_rows_ - 1))); // bottom center
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.min_x_, rows / 2.0, 0.0},
                                           std::make_pair(0, cam.num_grid_rows_ / 2 - 1))); // left center
    test_cases.emplace_back(std::make_pair(cv::KeyPoint{cam.img_bounds_.max_x_ - eps, rows / 2.0, 0.0},
                                           std::make_pair(cam.num_grid_cols_ - 1, cam.num_grid_rows_ / 2 - 1))); // right center

    // check
    for (const auto& test_case : test_cases) {
        // extract information
        const auto& keypt = test_case.first;
        const auto idx_x = test_case.second.first;
        const auto idx_y = test_case.second.second;
        // run the function
        int est_idx_x = -1;
        int est_idx_y = -1;
        const auto found = data::get_cell_indices(&cam, keypt, est_idx_x, est_idx_y);
        EXPECT_TRUE(found);
        EXPECT_EQ(est_idx_x, idx_x);
        EXPECT_EQ(est_idx_y, idx_y);
    }
}

TEST(common, valid_cases_2) {
    // create an example perspective camera
    constexpr unsigned int cols = 2000;
    constexpr unsigned int rows = 1000;
    auto cam = create_perspective_camera(cols, rows);
    // create keypoints and those grid IDs
    std::vector<std::pair<cv::KeyPoint, std::pair<int, int>>> test_cases;
    constexpr float eps = 0.01;
    // - corners of each cell
    for (unsigned int idx_x = 0; idx_x < cam.num_grid_cols_; ++idx_x) {
        for (unsigned int idx_y = 0; idx_y < cam.num_grid_rows_; ++idx_y) {
            // top left
            {
                const float x = idx_x * (1.0 / cam.inv_cell_width_) + eps;
                const float y = idx_y * (1.0 / cam.inv_cell_height_) + eps;
                test_cases.emplace_back(cv::KeyPoint{x, y, 0.0}, std::make_pair(idx_x, idx_y));
            }
            // top right
            {
                const float x = (idx_x + 1) * (1.0 / cam.inv_cell_width_) - eps;
                const float y = idx_y * (1.0 / cam.inv_cell_height_) + eps;
                test_cases.emplace_back(cv::KeyPoint{x, y, 0.0}, std::make_pair(idx_x, idx_y));
            }
            // bottom left
            {
                const float x = idx_x * (1.0 / cam.inv_cell_width_) + eps;
                const float y = (idx_y + 1) * (1.0 / cam.inv_cell_height_) - eps;
                test_cases.emplace_back(cv::KeyPoint{x, y, 0.0}, std::make_pair(idx_x, idx_y));
            }
            // bottom right
            {
                const float x = (idx_x + 1) * (1.0 / cam.inv_cell_width_) - eps;
                const float y = (idx_y + 1) * (1.0 / cam.inv_cell_height_) - eps;
                test_cases.emplace_back(cv::KeyPoint{x, y, 0.0}, std::make_pair(idx_x, idx_y));
            }
        }
    }

    // check
    for (const auto& test_case : test_cases) {
        // extract information
        const auto& keypt = test_case.first;
        const auto idx_x = test_case.second.first;
        const auto idx_y = test_case.second.second;
        // run the function
        int est_idx_x = -1;
        int est_idx_y = -1;
        const auto found = data::get_cell_indices(&cam, keypt, est_idx_x, est_idx_y);
        EXPECT_TRUE(found);
        EXPECT_EQ(est_idx_x, idx_x);
        EXPECT_EQ(est_idx_y, idx_y);
    }
}

TEST(common, invalid_cases) {
    // create an example perspective camera
    constexpr unsigned int cols = 2000;
    constexpr unsigned int rows = 1000;
    auto cam = create_perspective_camera(cols, rows, -0.1, 0.1);
    // create keypoints
    std::vector<cv::KeyPoint> test_cases;
    constexpr float eps = 0.01;
    // clang-format off
    // - invalid corners
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.min_x_ - eps, cam.img_bounds_.min_y_ - eps, 0.0}); // top left
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.max_x_, cam.img_bounds_.min_y_ - eps, 0.0}); // top right
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.min_x_ - eps, cam.img_bounds_.max_y_, 0.0}); // bottom left
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.max_x_, cam.img_bounds_.max_y_, 0.0}); // bottom right
    // - invalid edges
    test_cases.emplace_back(cv::KeyPoint{cols / 2.0, cam.img_bounds_.min_y_ - eps, 0.0}); // top center
    test_cases.emplace_back(cv::KeyPoint{cols / 2.0, cam.img_bounds_.max_y_, 0.0}); // bottom center
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.min_x_ - eps, rows / 2.0, 0.0}); // left center
    test_cases.emplace_back(cv::KeyPoint{cam.img_bounds_.max_x_, rows / 2.0, 0.0}); // right center
    // clang-format on

    // check
    for (const auto& test_case : test_cases) {
        // extract information
        const auto& keypt = test_case;
        // run the function
        int est_idx_x = -1;
        int est_idx_y = -1;
        const auto found = data::get_cell_indices(&cam, keypt, est_idx_x, est_idx_y);
        EXPECT_FALSE(found);
    }
}
