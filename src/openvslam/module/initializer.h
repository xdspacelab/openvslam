#ifndef OPENVSLAM_MODULE_INITIALIZER_H
#define OPENVSLAM_MODULE_INITIALIZER_H

#include "openvslam/data/frame.h"
#include "openvslam/initialize/base.h"

#include <memory>

namespace openvslam {

class config;

namespace data {
class frame;
class map_database;
class bow_database;
} // namespace data

namespace module {

// initializer state
enum class initializer_state_t {
    NotReady,
    Initializing,
    Wrong,
    Succeeded
};

class initializer {
public:
    initializer() = delete;

    //! Constructor
    initializer(const camera::setup_type_t setup_type,
                data::map_database* map_db, data::bow_database* bow_db,
                const YAML::Node& yaml_node);

    //! Destructor
    ~initializer();

    //! Reset initializer
    void reset();

    //! Get initialization state
    initializer_state_t get_state() const;

    //! Get keypoints of the initial frame
    std::vector<cv::KeyPoint> get_initial_keypoints() const;

    //! Get initial matches between the initial and current frames
    std::vector<int> get_initial_matches() const;

    //! Get the initial frame ID which succeeded in initialization
    unsigned int get_initial_frame_id() const;

    //! Initialize with the current frame
    bool initialize(data::frame& curr_frm);

private:
    //! camera setup type
    const camera::setup_type_t setup_type_;
    //! map database
    data::map_database* map_db_ = nullptr;
    //! BoW database
    data::bow_database* bow_db_ = nullptr;
    //! initializer status
    initializer_state_t state_ = initializer_state_t::NotReady;

    //! frame ID used for initialization (will be set after succeeded)
    unsigned int init_frm_id_ = 0;

    //-----------------------------------------
    // parameters

    //! max number of iterations of RANSAC (only for monocular initializer)
    const unsigned int num_ransac_iters_;
    //! min number of triangulated pts
    const unsigned int min_num_triangulated_;
    //! min parallax (only for monocular initializer)
    const float parallax_deg_thr_;
    //! reprojection error threshold (only for monocular initializer)
    const float reproj_err_thr_;
    //! max number of iterations of BA (only for monocular initializer)
    const unsigned int num_ba_iters_;
    //! initial scaling factor (only for monocular initializer)
    const float scaling_factor_;

    //-----------------------------------------
    // for monocular camera model

    //! Create initializer for monocular
    void create_initializer(data::frame& curr_frm);

    //! Try to initialize a map with monocular camera setup
    bool try_initialize_for_monocular(data::frame& curr_frm);

    //! Create an initial map with monocular camera setup
    bool create_map_for_monocular(data::frame& curr_frm);

    //! Scaling up or down a initial map
    void scale_map(const std::shared_ptr<data::keyframe>& init_keyfrm, const std::shared_ptr<data::keyframe>& curr_keyfrm, const double scale);

    //! initializer for monocular
    std::unique_ptr<initialize::base> initializer_ = nullptr;
    //! initial frame
    data::frame init_frm_;
    //! coordinates of previously matched points to perform area-based matching
    std::vector<cv::Point2f> prev_matched_coords_;
    //! initial matching indices (index: idx of initial frame, value: idx of current frame)
    std::vector<int> init_matches_;

    //-----------------------------------------
    // for stereo or RGBD camera model

    //! Try to initialize a map with stereo or RGBD camera setup
    bool try_initialize_for_stereo(data::frame& curr_frm);

    //! Create an initial map with stereo or RGBD camera setup
    bool create_map_for_stereo(data::frame& curr_frm);
};

} // namespace module
} // namespace openvslam

#endif // OPENVSLAM_MODULE_INITIALIZER_H
