#include "image_util.h"

#ifdef _MSC_VER
#include <io.h>
#else
#include <dirent.h>
#endif
#include <algorithm>
#include <stdexcept>

image_sequence::image_sequence(const std::string& img_dir_path, const double fps)
    : fps_(fps) {
#ifdef _MSC_VER
    // refine the path string
    std::string refined_img_dir_path;
    for (const auto c : img_dir_path) {
        if (c == '/' || c == '\\') {
            refined_img_dir_path.push_back('\\');
            refined_img_dir_path.push_back('\\');
        }
        else {
            refined_img_dir_path.push_back(c);
        }
    }
    refined_img_dir_path.push_back('\\');
    refined_img_dir_path.push_back('\\');
    refined_img_dir_path.push_back('*');
    // sweep the directory
    _finddata_t file;
    intptr_t dir;
    if ((dir = _findfirst(refined_img_dir_path.c_str(), &file)) == -1) {
        throw std::runtime_error("directory " + img_dir_path + " does not exist");
    }
    do {
        if ((file.attrib & _A_SUBDIR) != 0) {
            continue;
        }
        const std::string img_file_name = file.name;
        img_file_paths_.push_back(img_dir_path + "\\" + img_file_name);
    } while (_findnext(dir, &file) == 0);
    _findclose(dir);
#else
    // sweep the directory
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
        img_file_paths_.push_back(img_dir_path + "/" + img_file_name);
    }
    closedir(dir);
#endif

    std::sort(img_file_paths_.begin(), img_file_paths_.end());
}

std::vector<image_sequence::frame> image_sequence::get_frames() const {
    std::vector<frame> frames;
    for (unsigned int i = 0; i < img_file_paths_.size(); ++i) {
        frames.emplace_back(frame{img_file_paths_.at(i), (1.0 / fps_) * i});
    }
    return frames;
}
