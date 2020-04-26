#include "pangolin_viewer/color_scheme.h"

#include <cctype>

namespace pangolin_viewer {

color_scheme::color_scheme(const std::string& color_set_str) {
    if (stricmp(color_set_str, std::string("white"))) {
        set_color_as_white();
    }
    else if (stricmp(color_set_str, std::string("black"))) {
        set_color_as_black();
    }
    else if (stricmp(color_set_str, std::string("purple"))) {
        set_color_as_purple();
    }
    else {
        throw std::runtime_error("undefined color scheme: " + color_set_str);
    }
}

void color_scheme::set_color_as_white() {
    bg_ = {{1.0f, 1.0f, 1.0f, 1.0f}};
    grid_ = {{0.7f, 0.7f, 0.7f}};
    curr_cam_ = {{0.0f, 1.0f, 0.0f}};
    kf_line_ = {{0.0f, 0.0f, 1.0f}};
    graph_line_ = {{0.0f, 1.0f, 0.0f, 0.6f}};
    lm_ = {{0.0f, 0.0f, 0.0f}};
    local_lm_ = {{1.0f, 0.0f, 0.0f}};
}

void color_scheme::set_color_as_black() {
    bg_ = {{0.15f, 0.15f, 0.15f, 1.0f}};
    grid_ = {{0.3f, 0.3f, 0.3f}};
    curr_cam_ = {{0.7f, 0.7f, 1.0f}};
    kf_line_ = {{0.0f, 1.0f, 0.0f}};
    graph_line_ = {{0.7f, 0.7f, 1.0f, 0.4f}};
    lm_ = {{0.9f, 0.9f, 0.9f}};
    local_lm_ = {{1.0f, 0.1f, 0.1f}};
}

void color_scheme::set_color_as_purple() {
    bg_ = {{0.05f, 0.05f, 0.3f, 0.0f}};
    grid_ = {{0.3f, 0.3f, 0.3f}};
    curr_cam_ = {{0.7f, 0.7f, 1.0f}};
    kf_line_ = {{1.0f, 0.1f, 0.1f}};
    graph_line_ = {{0.7f, 0.7f, 1.0f, 0.4f}};
    lm_ = {{0.9f, 0.9f, 0.9f}};
    local_lm_ = {{0.0f, 1.0f, 0.0f}};
}

bool color_scheme::stricmp(const std::string& str1, const std::string& str2) {
    if (str1.size() != str2.size()) {
        return false;
    }

    return std::equal(str1.cbegin(), str1.cend(), str2.cbegin(), [](const char& lhs, const char& rhs) {
        return std::tolower(lhs) == std::tolower(rhs);
    });
}

} // namespace pangolin_viewer
