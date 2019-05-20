#ifndef IMAGE_UTIL_H
#define IMAGE_UTIL_H

#include <string>
#include <vector>

class image_sequence {
public:
    explicit image_sequence(const std::string& img_dir_path);

    virtual ~image_sequence() = default;

    std::vector<std::string> get_image_paths() const;

private:
    std::vector<std::string> img_paths_;
};

#endif // IMAGE_UTIL_H
