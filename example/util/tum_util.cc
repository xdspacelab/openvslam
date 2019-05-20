#include "tum_util.h"

#include <fstream>
#include <sstream>
#include <iomanip>

tum_mono_sequence::tum_mono_sequence(const std::string& seq_dir_path, const std::string& assoc_file_path) {
    std::ifstream ifs_assoc;
    ifs_assoc.open(assoc_file_path.c_str());

    while (!ifs_assoc.eof()) {
        std::string s;
        getline(ifs_assoc, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;

            double rgb_timestamp;
            std::string rgb_img_file_path;

            ss >> rgb_timestamp;
            ss >> rgb_img_file_path;

            rgb_img_file_paths_.push_back(seq_dir_path + "/" + rgb_img_file_path);
            timestamps_.push_back(rgb_timestamp);
        }
    }
    num_imgs_ = timestamps_.size();
}

std::vector<tum_mono_sequence::frame> tum_mono_sequence::get_frames() const {
    std::vector<frame> frames;
    for (unsigned int i = 0; i < num_imgs_; ++i) {
        frames.emplace_back(frame{rgb_img_file_paths_.at(i), timestamps_.at(i)});
    }
    return frames;
}

tum_rgbd_sequence::tum_rgbd_sequence(const std::string& seq_dir_path, const std::string& assoc_file_path) {
    std::ifstream ifs_assoc;
    ifs_assoc.open(assoc_file_path.c_str());

    while (!ifs_assoc.eof()) {
        std::string s;
        getline(ifs_assoc, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;

            double rgb_timestamp, depth_timestamp;
            std::string rgb_img_file_path, depth_img_file_path;

            ss >> rgb_timestamp;

            ss >> rgb_img_file_path;
            rgb_img_file_paths_.push_back(seq_dir_path + "/" + rgb_img_file_path);

            ss >> depth_timestamp;

            ss >> depth_img_file_path;
            depth_img_file_paths_.push_back(seq_dir_path + "/" + depth_img_file_path);

            timestamps_.push_back((rgb_timestamp + depth_timestamp) / 2.0);
        }
    }
    num_imgs_ = timestamps_.size();
}

std::vector<tum_rgbd_sequence::frame> tum_rgbd_sequence::get_frames() const {
    std::vector<frame> frames;
    for (unsigned int i = 0; i < num_imgs_; ++i) {
        frames.emplace_back(frame{rgb_img_file_paths_.at(i), depth_img_file_paths_.at(i), timestamps_.at(i)});
    }
    return frames;
}
