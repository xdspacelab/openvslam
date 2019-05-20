#include "kitti_util.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cassert>

kitti_mono_sequence::kitti_mono_sequence(const std::string& seq_dir_path) {
    // load timestamps
    const std::string timestamp_file_path = seq_dir_path + "/times.txt";
    std::ifstream ifs_timestamp;
    ifs_timestamp.open(timestamp_file_path.c_str());

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
    const std::string img_dir_path = seq_dir_path + "/image_0/";
    num_imgs_ = timestamps_.size();

    img_file_paths_.clear();
    for (unsigned int i = 0; i < num_imgs_; ++i) {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        img_file_paths_.push_back(img_dir_path + ss.str() + ".png");
    }
}

std::vector<kitti_mono_sequence::frame> kitti_mono_sequence::get_frames() const {
    std::vector<frame> frames;
    for (unsigned int i = 0; i < num_imgs_; ++i) {
        frames.emplace_back(frame{img_file_paths_.at(i), timestamps_.at(i)});
    }
    return frames;
}

kitti_stereo_sequence::kitti_stereo_sequence(const std::string& seq_dir_path) {
    // load timestamps
    const std::string timestamp_file_path = seq_dir_path + "/times.txt";
    std::ifstream ifs_timestamp;
    ifs_timestamp.open(timestamp_file_path.c_str());

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
    num_imgs_ = timestamps_.size();

    left_img_file_paths_.clear();
    right_img_file_paths_.clear();
    for (unsigned int i = 0; i < num_imgs_; ++i) {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        left_img_file_paths_.push_back(left_img_dir_path + ss.str() + ".png");
        right_img_file_paths_.push_back(right_img_dir_path + ss.str() + ".png");
    }
}

std::vector<kitti_stereo_sequence::frame> kitti_stereo_sequence::get_frames() const {
    std::vector<frame> frames;
    for (unsigned int i = 0; i < num_imgs_; ++i) {
        frames.emplace_back(frame{left_img_file_paths_.at(i), right_img_file_paths_.at(i), timestamps_.at(i)});
    }
    return frames;
}
