#include "kitti_util.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cassert>

kitti_sequence::kitti_sequence(const std::string& seq_dir_path) {
    // load timestamps
    const std::string timestamp_file_path = seq_dir_path + "/times.txt";
    std::ifstream ifs_timestamp;
    ifs_timestamp.open(timestamp_file_path.c_str());
    if (!ifs_timestamp) {
        throw std::runtime_error("Could not load a timestamp file from " + timestamp_file_path);
    }

    timestamps_.clear();
    while (!ifs_timestamp.eof()) {
        std::string s;
        getline(ifs_timestamp, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            double timestamp;
            ss >> timestamp;
            timestamps_.push_back(timestamp);
        }
    }

    ifs_timestamp.close();

    // load image file paths
    const std::string left_img_dir_path = seq_dir_path + "/image_0/";
    const std::string right_img_dir_path = seq_dir_path + "/image_1/";

    left_img_file_paths_.clear();
    right_img_file_paths_.clear();
    for (unsigned int i = 0; i < timestamps_.size(); ++i) {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        left_img_file_paths_.push_back(left_img_dir_path + ss.str() + ".png");
        right_img_file_paths_.push_back(right_img_dir_path + ss.str() + ".png");
    }
}

std::vector<kitti_sequence::frame> kitti_sequence::get_frames() const {
    std::vector<frame> frames;
    assert(timestamps_.size() == left_img_file_paths_.size());
    assert(timestamps_.size() == right_img_file_paths_.size());
    assert(left_img_file_paths_.size() == right_img_file_paths_.size());
    for (unsigned int i = 0; i < timestamps_.size(); ++i) {
        frames.emplace_back(frame{left_img_file_paths_.at(i), right_img_file_paths_.at(i), timestamps_.at(i)});
    }
    return frames;
}
