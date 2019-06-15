#include "tum_rgbd_util.h"

#include <cassert>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <dirent.h>
#include <algorithm>

tum_rgbd_sequence::tum_rgbd_sequence(const std::string& seq_dir_path, const double min_timediff_thr) {
    // listing up the files in rgb/ and depth/ directories
    const auto rgb_img_infos = acquire_image_information(seq_dir_path + "/rgb/");
    const auto depth_img_infos = acquire_image_information(seq_dir_path + "/depth/");

    // find the nearest depth frame for each of the RGB frames
    for (const auto& rgb_img_info : rgb_img_infos) {
        // untie RGB frame information
        const auto& rgb_img_timestamp = rgb_img_info.timestamp();
        const auto& rgb_img_file_path = rgb_img_info.img_file_path();

        // nearest depth frame information
        auto nearest_depth_img_timestamp = depth_img_infos.begin()->timestamp();
        auto nearest_depth_img_file_path = depth_img_infos.begin()->img_file_path();
        double min_timediff = std::abs(rgb_img_timestamp - nearest_depth_img_timestamp);

        // calc time diff and find the nearest depth frame
        for (const auto& depth_img_info : depth_img_infos) {
            // untie RGB frame information
            const auto& depth_img_timestamp = depth_img_info.timestamp();
            const auto& depth_img_file_path = depth_img_info.img_file_path();
            // calc time diff
            const auto timediff = std::abs(rgb_img_timestamp - depth_img_timestamp);
            // find the nearest depth frame
            if (timediff < min_timediff) {
                min_timediff = timediff;
                nearest_depth_img_timestamp = depth_img_timestamp;
                nearest_depth_img_file_path = depth_img_file_path;
            }
        }

        // reject if the time diff is over the threshold
        if (min_timediff_thr < min_timediff) {
            continue;
        }

        timestamps_.push_back((rgb_img_timestamp + nearest_depth_img_timestamp) / 2.0);
        rgb_img_file_paths_.push_back(rgb_img_file_path);
        depth_img_file_paths_.push_back(nearest_depth_img_file_path);
    }
}

std::vector<tum_rgbd_sequence::frame> tum_rgbd_sequence::get_frames() const {
    std::vector<frame> frames;
    assert(timestamps_.size() == rgb_img_file_paths_.size());
    assert(timestamps_.size() == depth_img_file_paths_.size());
    assert(rgb_img_file_paths_.size() == depth_img_file_paths_.size());
    for (unsigned int i = 0; i < timestamps_.size(); ++i) {
        frames.emplace_back(frame{rgb_img_file_paths_.at(i), depth_img_file_paths_.at(i), timestamps_.at(i)});
    }
    return frames;
}

std::vector<tum_rgbd_sequence::img_info> tum_rgbd_sequence::acquire_image_information(const std::string& img_dir_path) const {
    std::vector<tum_rgbd_sequence::img_info> img_infos;

    DIR* dir;
    if ((dir = opendir(img_dir_path.c_str())) == nullptr) {
        throw std::runtime_error("directory " + img_dir_path + " does not exist");
    }
    dirent* dp;
    for (dp = readdir(dir); dp != nullptr; dp = readdir(dir)) {
        const std::string img_file_name = dp->d_name;
        if (img_file_name == "." || img_file_name == "..") {
            continue;
        }
        const std::string timestamp_str(img_file_name.begin(), img_file_name.end() - 4);
        const auto timestamp = std::stod(timestamp_str);
        img_infos.emplace_back(img_info{timestamp, img_dir_path + "/" + img_file_name});
    }
    closedir(dir);

    std::sort(img_infos.begin(), img_infos.end());

    return img_infos;
}
