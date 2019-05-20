#ifndef TUM_UTIL_H
#define TUM_UTIL_H

#include <string>
#include <vector>

class tum_mono_sequence {
public:
    struct frame {
        frame(const std::string& rgb_img_path, const double timestamp)
                : rgb_img_path_(rgb_img_path), timestamp_(timestamp) {};

        const std::string rgb_img_path_;
        const double timestamp_;
    };

    explicit tum_mono_sequence(const std::string& seq_dir_path, const std::string& assoc_file_path);

    virtual ~tum_mono_sequence() = default;

    std::vector<frame> get_frames() const;

private:
    std::vector<double> timestamps_;
    std::vector<std::string> rgb_img_file_paths_;
    unsigned int num_imgs_;
};

class tum_rgbd_sequence {
public:
    struct frame {
        frame(const std::string& rgb_img_path, const std::string& depth_img_path, const double timestamp)
                : rgb_img_path_(rgb_img_path), depth_img_path_(depth_img_path), timestamp_(timestamp) {};

        const std::string rgb_img_path_;
        const std::string depth_img_path_;
        const double timestamp_;
    };

    explicit tum_rgbd_sequence(const std::string& seq_dir_path, const std::string& assoc_file_path);

    virtual ~tum_rgbd_sequence() = default;

    std::vector<frame> get_frames() const;

private:
    std::vector<double> timestamps_;
    std::vector<std::string> rgb_img_file_paths_;
    std::vector<std::string> depth_img_file_paths_;
    unsigned int num_imgs_;
};

#endif // TUM_UTIL_H
