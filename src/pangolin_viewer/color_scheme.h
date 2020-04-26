#ifndef PANGOLIN_VIEWER_COLOR_SCHEME_H
#define PANGOLIN_VIEWER_COLOR_SCHEME_H

#include <array>
#include <string>
#include <cctype>

namespace pangolin_viewer {

class color_scheme {
public:
    explicit color_scheme(const std::string& color_set_str);

    virtual ~color_scheme() = default;

    //! background color
    std::array<float, 4> bg_{};
    //! grid color
    std::array<float, 3> grid_{};
    //! current camera color
    std::array<float, 3> curr_cam_{};
    //! keyframe line color
    std::array<float, 3> kf_line_{};
    //! graph edge line color
    std::array<float, 4> graph_line_{};
    //! landmark color
    std::array<float, 3> lm_{};
    //! local_landmark color
    std::array<float, 3> local_lm_{};

private:
    void set_color_as_white();

    void set_color_as_black();

    void set_color_as_purple();

    static bool stricmp(const std::string& str1, const std::string& str2);
};

} // namespace pangolin_viewer

#endif // PANGOLIN_VIEWER_COLOR_SCHEME_H
