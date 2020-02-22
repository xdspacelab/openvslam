#ifndef OPENVSLAM_DATA_COMMON_H
#define OPENVSLAM_DATA_COMMON_H

#include "openvslam/type.h"
#include "openvslam/camera/base.h"

#include <opencv2/core.hpp>
#include <nlohmann/json_fwd.hpp>

namespace openvslam {
namespace data {

nlohmann::json convert_rotation_to_json(const Mat33_t& rot_cw);

Mat33_t convert_json_to_rotation(const nlohmann::json& json_rot_cw);

nlohmann::json convert_translation_to_json(const Vec3_t& trans_cw);

Vec3_t convert_json_to_translation(const nlohmann::json& json_trans_cw);

nlohmann::json convert_keypoints_to_json(const std::vector<cv::KeyPoint>& keypts);

std::vector<cv::KeyPoint> convert_json_to_keypoints(const nlohmann::json& json_keypts);

nlohmann::json convert_undistorted_to_json(const std::vector<cv::KeyPoint>& undist_keypts);

std::vector<cv::KeyPoint> convert_json_to_undistorted(const nlohmann::json& json_undist_keypts, const std::vector<cv::KeyPoint>& keypts = {});

nlohmann::json convert_descriptors_to_json(const cv::Mat& descriptors);

cv::Mat convert_json_to_descriptors(const nlohmann::json& json_descriptors);

/**
 * Assign all keypoints to cells to accelerate projection matching
 * @param camera
 * @param undist_keypts
 * @param keypt_indices_in_cells
 */
void assign_keypoints_to_grid(camera::base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                              std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells);

/**
 * Assign all keypoints to cells to accelerate projection matching
 * @param camera
 * @param undist_keypts
 * @return
 */
auto assign_keypoints_to_grid(camera::base* camera, const std::vector<cv::KeyPoint>& undist_keypts)
    -> std::vector<std::vector<std::vector<unsigned int>>>;

/**
 * Get x-y index of the cell in which the specified keypoint is assigned
 * @param camera
 * @param keypt
 * @param cell_idx_x
 * @param cell_idx_y
 * @return
 */
inline bool get_cell_indices(camera::base* camera, const cv::KeyPoint& keypt, int& cell_idx_x, int& cell_idx_y) {
    cell_idx_x = cvFloor((keypt.pt.x - camera->img_bounds_.min_x_) * camera->inv_cell_width_);
    cell_idx_y = cvFloor((keypt.pt.y - camera->img_bounds_.min_y_) * camera->inv_cell_height_);
    return (0 <= cell_idx_x && cell_idx_x < static_cast<int>(camera->num_grid_cols_)
            && 0 <= cell_idx_y && cell_idx_y < static_cast<int>(camera->num_grid_rows_));
}

/**
 * Get keypoint indices in cell(s) in which the specified point is located
 * @param camera
 * @param undist_keypts
 * @param keypt_indices_in_cells
 * @param ref_x
 * @param ref_y
 * @param margin
 * @param min_level
 * @param max_level
 * @return
 */
std::vector<unsigned int> get_keypoints_in_cell(camera::base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                                                const std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells,
                                                const float ref_x, const float ref_y, const float margin,
                                                const int min_level = -1, const int max_level = -1);

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_COMMON_H
