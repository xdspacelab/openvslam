#include "image_util.h"

#include <dirent.h>
#include <algorithm>

image_sequence::image_sequence(const std::string& img_dir_path, const double fps) : fps_(fps) {
    DIR* dir;
    if ((dir = opendir(img_dir_path.c_str())) == nullptr) {

    }
    dirent* dp;
    for (dp = readdir(dir); dp != nullptr; dp = readdir(dir)) {
        img_file_paths_.push_back(img_dir_path + "/" + std::string(dp->d_name));
    }
    closedir(dir);

    std::sort(img_file_paths_.begin(), img_file_paths_.end());
}

std::vector<image_sequence::frame> image_sequence::get_frames() const {
    std::vector<frame> frames;
    for (unsigned int i = 0; i < img_file_paths_.size(); ++i) {
        frames.emplace_back(frame{img_file_paths_.at(i), (1.0 / fps_) * i});
    }
    return frames;
}
