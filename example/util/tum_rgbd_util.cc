#include "tum_rgbd_util.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <algorithm>
#include <cmath>

tum_rgbd_sequence::tum_rgbd_sequence(const std::string& seq_dir_path, const double min_timediff_thr) {
    // listing up the files in rgb/ and depth/ directories
    const auto rgb_img_infos = acquire_image_information(seq_dir_path, seq_dir_path + "/rgb.txt");
    const auto depth_img_infos = acquire_image_information(seq_dir_path, seq_dir_path + "/depth.txt");

    // find the nearest depth frame for each of the RGB frames
    for (const auto& rgb_img_info : rgb_img_infos) {
        // untie RGB frame information
        const auto& rgb_img_timestamp = rgb_img_info.timestamp_;
        const auto& rgb_img_file_path = rgb_img_info.img_file_path_;

        // nearest depth frame information
        auto nearest_depth_img_timestamp = depth_img_infos.begin()->timestamp_;
        auto nearest_depth_img_file_path = depth_img_infos.begin()->img_file_path_;
        double min_timediff = std::abs(rgb_img_timestamp - nearest_depth_img_timestamp);

        // calc time diff and find the nearest depth frame
        for (const auto& depth_img_info : depth_img_infos) {
            // untie RGB frame information
            const auto& depth_img_timestamp = depth_img_info.timestamp_;
            const auto& depth_img_file_path = depth_img_info.img_file_path_;
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

std::vector<tum_rgbd_sequence::img_info> tum_rgbd_sequence::acquire_image_information(const std::string& seq_dir_path,
                                                                                      const std::string& timestamp_file_path) const {
    std::vector<tum_rgbd_sequence::img_info> img_infos;

    // load timestamps
    std::ifstream ifs_timestamp;
    ifs_timestamp.open(timestamp_file_path.c_str());
    if (!ifs_timestamp) {
        throw std::runtime_error("Could not load a timestamp file from " + timestamp_file_path);
    }

    // load header row
    std::string s;
    getline(ifs_timestamp, s);
    getline(ifs_timestamp, s);
    getline(ifs_timestamp, s);

    while (!ifs_timestamp.eof()) {
        getline(ifs_timestamp, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            double timestamp;
            std::string img_file_name;
            ss >> timestamp >> img_file_name;
            img_infos.emplace_back(img_info{timestamp, seq_dir_path + "/" + img_file_name});
        }
    }

    ifs_timestamp.close();

    return img_infos;
}
