#include "euroc_util.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <algorithm>

euroc_sequence::euroc_sequence(const std::string& seq_dir_path) {
    const std::string timestamp_file_path = seq_dir_path + "/cam0/data.csv";
    const std::string left_img_dir_path = seq_dir_path + "/cam0/data/";
    const std::string right_img_dir_path = seq_dir_path + "/cam1/data/";

    timestamps_.clear();
    left_img_file_paths_.clear();
    right_img_file_paths_.clear();

    // load timestamps
    std::ifstream ifs_timestamp;
    ifs_timestamp.open(timestamp_file_path.c_str());
    if (!ifs_timestamp) {
        throw std::runtime_error("Could not load a timestamp file from " + timestamp_file_path);
    }

    // load header row
    std::string s;
    getline(ifs_timestamp, s);

    while (!ifs_timestamp.eof()) {
        getline(ifs_timestamp, s);
        std::replace(s.begin(), s.end(), ',', ' ');
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            unsigned long long timestamp;
            ss >> timestamp;
            timestamps_.push_back(timestamp / static_cast<double>(1E9));
            left_img_file_paths_.push_back(left_img_dir_path + std::to_string(timestamp) + ".png");
            right_img_file_paths_.push_back(right_img_dir_path + std::to_string(timestamp) + ".png");
        }
    }

    ifs_timestamp.close();
}

std::vector<euroc_sequence::frame> euroc_sequence::get_frames() const {
    std::vector<frame> frames;
    assert(timestamps_.size() == left_img_file_paths_.size());
    assert(timestamps_.size() == right_img_file_paths_.size());
    assert(left_img_file_paths_.size() == right_img_file_paths_.size());
    for (unsigned int i = 0; i < timestamps_.size(); ++i) {
        frames.emplace_back(frame{left_img_file_paths_.at(i), right_img_file_paths_.at(i), timestamps_.at(i)});
    }
    return frames;
}
