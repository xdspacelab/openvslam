#ifndef PANGOLIN_VIEWER_VIEWER_H
#define PANGOLIN_VIEWER_VIEWER_H

#include "pangolin_viewer/color_scheme.h"

#include "openvslam/type.h"

#include <memory>
#include <mutex>

#include <pangolin/pangolin.h>

namespace openvslam {

class config;
class system;

namespace publish {
class frame_publisher;
class map_publisher;
} // namespace publish

} // namespace openvslam

namespace pangolin_viewer {

class viewer {
public:
    /**
     * Constructor
     * @param cfg
     * @param system
     * @param frame_publisher
     * @param map_publisher
     */
    viewer(const std::shared_ptr<openvslam::config>& cfg, openvslam::system* system,
           const std::shared_ptr<openvslam::publish::frame_publisher>& frame_publisher,
           const std::shared_ptr<openvslam::publish::map_publisher>& map_publisher);

    /**
     * Main loop for window refresh
     */
    void run();

    /**
     * Request to terminate the viewer
     * (NOTE: this function does not wait for terminate)
     */
    void request_terminate();

    /**
     * Check if the viewer is terminated or not
     * @return whether the viewer is terminated or not
     */
    bool is_terminated();

private:
    /**
     * Create menu panel
     */
    void create_menu_panel();

    /**
     * Follow to the specified camera pose
     * @param gl_cam_pose_wc
     */
    void follow_camera(const pangolin::OpenGlMatrix& gl_cam_pose_wc);

    /**
     * Get the current camera pose via the map publisher
     * @return
     */
    pangolin::OpenGlMatrix get_current_cam_pose();

    /**
     * Draw the horizontal grid
     */
    void draw_horizontal_grid();

    /**
     * Draw the current camera pose
     * @param gl_cam_pose_wc
     */
    void draw_current_cam_pose(const pangolin::OpenGlMatrix& gl_cam_pose_wc);

    /**
     * Get and draw keyframes via the map publisher
     */
    void draw_keyframes();

    /**
     * Get and draw landmarks via the map publisher
     */
    void draw_landmarks();

    /**
     * Draw the camera frustum of the specified camera pose
     * @param gl_cam_pose_wc
     * @param width
     */
    void draw_camera(const pangolin::OpenGlMatrix& gl_cam_pose_wc, const float width) const;

    /**
     * Draw the camera frustum of the specified camera pose
     * @param gl_cam_pose_wc
     * @param width
     */
    void draw_camera(const openvslam::Mat44_t& cam_pose_wc, const float width) const;

    /**
     * Draw a frustum of a camera
     * @param w
     */
    void draw_frustum(const float w) const;

    /**
     * Draw a line between two 3D points
     */
    void draw_line(const float x1, const float y1, const float z1,
                   const float x2, const float y2, const float z2) const;

    /**
     * Reset the states
     */
    void reset();

    /**
     * Check state transition
     */
    void check_state_transition();

    //! system
    openvslam::system* system_;
    //! frame publisher
    const std::shared_ptr<openvslam::publish::frame_publisher> frame_publisher_;
    //! map publisher
    const std::shared_ptr<openvslam::publish::map_publisher> map_publisher_;

    const unsigned int interval_ms_;

    const float viewpoint_x_, viewpoint_y_, viewpoint_z_, viewpoint_f_;

    const float keyfrm_size_;
    const float keyfrm_line_width_;
    const float graph_line_width_;
    const float point_size_;
    const float camera_size_;
    const float camera_line_width_;

    const color_scheme cs_;

    // menu panel
    std::unique_ptr<pangolin::Var<bool>> menu_follow_camera_;
    std::unique_ptr<pangolin::Var<bool>> menu_grid_;
    std::unique_ptr<pangolin::Var<bool>> menu_show_keyfrms_;
    std::unique_ptr<pangolin::Var<bool>> menu_show_lms_;
    std::unique_ptr<pangolin::Var<bool>> menu_show_local_map_;
    std::unique_ptr<pangolin::Var<bool>> menu_show_graph_;
    std::unique_ptr<pangolin::Var<bool>> menu_mapping_mode_;
    std::unique_ptr<pangolin::Var<bool>> menu_loop_detection_mode_;
    std::unique_ptr<pangolin::Var<bool>> menu_pause_;
    std::unique_ptr<pangolin::Var<bool>> menu_reset_;
    std::unique_ptr<pangolin::Var<bool>> menu_terminate_;
    std::unique_ptr<pangolin::Var<float>> menu_frm_size_;
    std::unique_ptr<pangolin::Var<float>> menu_lm_size_;

    // camera renderer
    std::unique_ptr<pangolin::OpenGlRenderState> s_cam_;

    // current state
    bool follow_camera_ = true;
    bool mapping_mode_ = true;
    bool loop_detection_mode_ = true;

    // viewer appearance
    const std::string map_viewer_name_{"PangolinViewer: Map Viewer"};
    const std::string frame_viewer_name_{"PangolinViewer: Frame Viewer"};
    static constexpr float map_viewer_width_ = 1024;
    static constexpr float map_viewer_height_ = 768;

    //-----------------------------------------
    // management for terminate process

    //! mutex for access to terminate procedure
    mutable std::mutex mtx_terminate_;

    /**
     * Check if termination is requested or not
     * @return
     */
    bool terminate_is_requested();

    /**
     * Raise the flag which indicates the main loop has been already terminated
     */
    void terminate();

    //! flag which indicates termination is requested or not
    bool terminate_is_requested_ = false;
    //! flag which indicates whether the main loop is terminated or not
    bool is_terminated_ = true;
};

inline void viewer::draw_line(const float x1, const float y1, const float z1,
                              const float x2, const float y2, const float z2) const {
    glVertex3f(x1, y1, z1);
    glVertex3f(x2, y2, z2);
}

} // namespace pangolin_viewer

#endif // PANGOLIN_VIEWER_VIEWER_H
