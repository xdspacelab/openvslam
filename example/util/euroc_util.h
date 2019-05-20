#ifndef EUROC_UTIL_H
#define EUROC_UTIL_H

#include <string>
#include <vector>

class euroc_mono_sequence {
public:
    struct frame {
        frame(const std::string& img_path, const double timestamp)
                : img_path_(img_path), timestamp_(timestamp) {};

        const std::string img_path_;
        const double timestamp_;
    };

    explicit euroc_mono_sequence(const std::string& seq_dir_path);

    virtual ~euroc_mono_sequence() = default;

    std::vector<frame> get_frames() const;

private:
    std::vector<double> timestamps_;
    std::vector<std::string> img_file_paths_;
    unsigned int num_imgs_;
};

class euroc_stereo_sequence {
public:
    struct frame {
        frame(const std::string& left_img_path, const std::string& right_img_path, const double timestamp)
                : left_img_path_(left_img_path), right_img_path_(right_img_path), timestamp_(timestamp) {};

        const std::string left_img_path_;
        const std::string right_img_path_;
        const double timestamp_;
    };

    explicit euroc_stereo_sequence(const std::string& seq_dir_path);

    virtual ~euroc_stereo_sequence() = default;

    std::vector<frame> get_frames() const;

private:
    std::vector<double> timestamps_;
    std::vector<std::string> left_img_file_paths_;
    std::vector<std::string> right_img_file_paths_;
    unsigned int num_imgs_;
};

#endif // EUROC_UTIL_H
