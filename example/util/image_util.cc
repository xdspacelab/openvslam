#include "image_util.h"

#include <dirent.h>
#include <algorithm>

image_sequence::image_sequence(const std::string& img_dir_path) {
    DIR* dir;
    if ((dir = opendir(img_dir_path.c_str())) == nullptr) {

    }
    dirent* dp;
    for (dp = readdir(dir); dp != nullptr; dp = readdir(dir)) {
        img_paths_.push_back(img_dir_path + "/" + std::string(dp->d_name));
    }
    closedir(dir);

    std::sort(img_paths_.begin(), img_paths_.end());
}

std::vector<std::string> image_sequence::get_image_paths() const {
    return img_paths_;
}
