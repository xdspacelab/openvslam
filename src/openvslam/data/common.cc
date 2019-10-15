#include "openvslam/data/common.h"

namespace openvslam {
namespace data {

nlohmann::json convert_rotation_to_json(const Mat33_t& rot_cw) {
    const Quat_t quat_cw(rot_cw);
    return {quat_cw.x(), quat_cw.y(), quat_cw.z(), quat_cw.w()};
}

Mat33_t convert_json_to_rotation(const nlohmann::json& json_rot_cw) {
    const Quat_t quat_cw(json_rot_cw.get<std::vector<double>>().data());
    return quat_cw.toRotationMatrix();
}

nlohmann::json convert_translation_to_json(const Vec3_t& trans_cw) {
    return {trans_cw(0), trans_cw(1), trans_cw(2)};
}

Vec3_t convert_json_to_translation(const nlohmann::json& json_trans_cw) {
    const Vec3_t trans_cw(json_trans_cw.get<std::vector<double>>().data());
    return trans_cw;
}

nlohmann::json convert_keypoints_to_json(const std::vector<cv::KeyPoint>& keypts) {
    std::vector<nlohmann::json> json_keypts(keypts.size());
    for (unsigned int idx = 0; idx < keypts.size(); ++idx) {
        json_keypts.at(idx) = {{"pt", {keypts.at(idx).pt.x, keypts.at(idx).pt.y}},
                               {"ang", keypts.at(idx).angle},
                               {"oct", static_cast<unsigned int>(keypts.at(idx).octave)}};
    }
    return std::move(json_keypts);
}

std::vector<cv::KeyPoint> convert_json_to_keypoints(const nlohmann::json& json_keypts) {
    std::vector<cv::KeyPoint> keypts(json_keypts.size());
    for (unsigned int idx = 0; idx < json_keypts.size(); ++idx) {
        const auto& json_keypt = json_keypts.at(idx);
        keypts.at(idx) = cv::KeyPoint(json_keypt.at("pt").at(0).get<float>(),
                                      json_keypt.at("pt").at(1).get<float>(),
                                      0,
                                      json_keypt.at("ang").get<float>(),
                                      0,
                                      json_keypt.at("oct").get<unsigned int>(),
                                      -1);
    }
    return keypts;
}

nlohmann::json convert_undistorted_to_json(const std::vector<cv::KeyPoint>& undist_keypts) {
    std::vector<nlohmann::json> json_undist_keypts(undist_keypts.size());
    for (unsigned int idx = 0; idx < undist_keypts.size(); ++idx) {
        json_undist_keypts.at(idx) = {undist_keypts.at(idx).pt.x, undist_keypts.at(idx).pt.y};
    }
    return json_undist_keypts;
}

std::vector<cv::KeyPoint> convert_json_to_undistorted(const nlohmann::json& json_undist_keypts, const std::vector<cv::KeyPoint>& keypts) {
    auto undist_keypts = (keypts.empty() ? std::vector<cv::KeyPoint>(json_undist_keypts.size()) : keypts);
    assert(undist_keypts.size() == json_undist_keypts.size());
    for (unsigned int idx = 0; idx < json_undist_keypts.size(); ++idx) {
        const auto& json_undist_keypt = json_undist_keypts.at(idx);
        undist_keypts.at(idx).pt.x = json_undist_keypt.at(0).get<float>();
        undist_keypts.at(idx).pt.y = json_undist_keypt.at(1).get<float>();
    }
    return undist_keypts;
}

nlohmann::json convert_descriptors_to_json(const cv::Mat& descriptors) {
    std::vector<nlohmann::json> json_descriptors(descriptors.rows);
    for (int idx = 0; idx < descriptors.rows; ++idx) {
        const cv::Mat& desc = descriptors.row(idx);
        const auto* p = desc.ptr<uint32_t>();
        std::vector<nlohmann::json> numbered_desc(8);
        for (unsigned int j = 0; j < 8; ++j, ++p) {
            numbered_desc.at(j) = *p;
        }
        json_descriptors.at(idx) = numbered_desc;
    }
    return json_descriptors;
}

cv::Mat convert_json_to_descriptors(const nlohmann::json& json_descriptors) {
    cv::Mat descriptors(json_descriptors.size(), 32, CV_8U);
    for (unsigned int idx = 0; idx < json_descriptors.size(); ++idx) {
        const auto& json_descriptor = json_descriptors.at(idx);
        auto p = descriptors.row(idx).ptr<uint32_t>();
        for (unsigned int i = 0; i < 8; ++i, ++p) {
            *p = json_descriptor.at(i).get<uint32_t>();
        }
    }
    return descriptors;
}

void assign_keypoints_to_grid(camera::base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                              std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells) {
    // Pre-allocate memory
    const unsigned int num_keypts = undist_keypts.size();
    const unsigned int num_to_reserve = 0.5 * num_keypts / (camera->num_grid_cols_ * camera->num_grid_rows_);
    keypt_indices_in_cells.resize(camera->num_grid_cols_);
    for (auto& keypt_indices_in_row : keypt_indices_in_cells) {
        keypt_indices_in_row.resize(camera->num_grid_rows_);
        for (auto& keypt_indices_in_cell : keypt_indices_in_row) {
            keypt_indices_in_cell.reserve(num_to_reserve);
        }
    }

    // Calculate cell position and store
    for (unsigned int idx = 0; idx < num_keypts; ++idx) {
        const auto& keypt = undist_keypts.at(idx);
        int cell_idx_x, cell_idx_y;
        if (get_cell_indices(camera, keypt, cell_idx_x, cell_idx_y)) {
            keypt_indices_in_cells.at(cell_idx_x).at(cell_idx_y).push_back(idx);
        }
    }
}

auto assign_keypoints_to_grid(camera::base* camera, const std::vector<cv::KeyPoint>& undist_keypts)
    -> std::vector<std::vector<std::vector<unsigned int>>> {
    std::vector<std::vector<std::vector<unsigned int>>> keypt_indices_in_cells;
    assign_keypoints_to_grid(camera, undist_keypts, keypt_indices_in_cells);
    return keypt_indices_in_cells;
}

std::vector<unsigned int> get_keypoints_in_cell(camera::base* camera, const std::vector<cv::KeyPoint>& undist_keypts,
                                                const std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells,
                                                const float ref_x, const float ref_y, const float margin,
                                                const int min_level, const int max_level) {
    std::vector<unsigned int> indices;
    indices.reserve(undist_keypts.size());

    const int min_cell_idx_x = std::max(0, cvFloor((ref_x - camera->img_bounds_.min_x_ - margin) * camera->inv_cell_width_));
    if (static_cast<int>(camera->num_grid_cols_) <= min_cell_idx_x) {
        return indices;
    }

    const int max_cell_idx_x = std::min(static_cast<int>(camera->num_grid_cols_ - 1), cvCeil((ref_x - camera->img_bounds_.min_x_ + margin) * camera->inv_cell_width_));
    if (max_cell_idx_x < 0) {
        return indices;
    }

    const int min_cell_idx_y = std::max(0, cvFloor((ref_y - camera->img_bounds_.min_y_ - margin) * camera->inv_cell_height_));
    if (static_cast<int>(camera->num_grid_rows_) <= min_cell_idx_y) {
        return indices;
    }

    const int max_cell_idx_y = std::min(static_cast<int>(camera->num_grid_rows_ - 1), cvCeil((ref_y - camera->img_bounds_.min_y_ + margin) * camera->inv_cell_height_));
    if (max_cell_idx_y < 0) {
        return indices;
    }

    const bool check_level = (0 < min_level) || (0 <= max_level);

    for (int cell_idx_x = min_cell_idx_x; cell_idx_x <= max_cell_idx_x; ++cell_idx_x) {
        for (int cell_idx_y = min_cell_idx_y; cell_idx_y <= max_cell_idx_y; ++cell_idx_y) {
            const auto& keypt_indices_in_cell = keypt_indices_in_cells.at(cell_idx_x).at(cell_idx_y);
            if (keypt_indices_in_cell.empty()) {
                continue;
            }

            for (unsigned int idx : keypt_indices_in_cell) {
                const auto& undist_keypt = undist_keypts.at(idx);

                if (check_level) {
                    if (undist_keypt.octave < min_level) {
                        continue;
                    }
                    if (0 <= max_level && max_level < undist_keypt.octave) {
                        continue;
                    }
                }

                const float dist_x = undist_keypt.pt.x - ref_x;
                const float dist_y = undist_keypt.pt.y - ref_y;

                if (std::abs(dist_x) < margin && std::abs(dist_y) < margin) {
                    indices.push_back(idx);
                }
            }
        }
    }

    return indices;
}

} // namespace data
} // namespace openvslam
