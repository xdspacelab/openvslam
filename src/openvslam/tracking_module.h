#ifndef OPENVSLAM_TRACKING_MODULE_H
#define OPENVSLAM_TRACKING_MODULE_H

#include "openvslam/type.h"
#include "openvslam/data/frame.h"
#include "openvslam/module/initializer.h"
#include "openvslam/module/relocalizer.h"
#include "openvslam/module/keyframe_inserter.h"
#include "openvslam/module/frame_tracker.h"

#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace openvslam {

class system;
class mapping_module;
class global_optimization_module;

namespace data {
class map_database;
class bow_database;
} // namespace data

namespace feature {
class orb_extractor;
} // namespace feature

// tracker state
enum class tracker_state_t {
    NotInitialized,
    Initializing,
    Tracking,
    Lost
};

class tracking_module {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    tracking_module(const std::shared_ptr<config>& cfg, system* system, data::map_database* map_db,
                    data::bow_vocabulary* bow_vocab, data::bow_database* bow_db);

    //! Destructor
    ~tracking_module();

    //! Set the mapping module
    void set_mapping_module(mapping_module* mapper);

    //! Set the global optimization module
    void set_global_optimization_module(global_optimization_module* global_optimizer);

    //-----------------------------------------
    // interfaces

    //! Set mapping module status
    void set_mapping_module_status(const bool mapping_is_enabled);

    //! Get mapping module status
    bool get_mapping_module_status() const;

    //! Get the keypoints of the initial frame
    std::vector<cv::KeyPoint> get_initial_keypoints() const;

    //! Get the keypoint matches between the initial frame and the current frame
    std::vector<int> get_initial_matches() const;

    //! Track a monocular frame
    //! (NOTE: distorted images are acceptable if calibrated)
    Mat44_t track_monocular_image(const cv::Mat& img, const double timestamp, const cv::Mat& mask = cv::Mat{});

    //! Track a stereo frame
    //! (Note: Left and Right images must be stereo-rectified)
    Mat44_t track_stereo_image(const cv::Mat& left_img_rect, const cv::Mat& right_img_rect, const double timestamp, const cv::Mat& mask = cv::Mat{});

    //! Track an RGBD frame
    //! (Note: RGB and Depth images must be aligned)
    Mat44_t track_RGBD_image(const cv::Mat& img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask = cv::Mat{});

    //-----------------------------------------
    // management for reset process

    //! Reset the databases
    void reset();

    //-----------------------------------------
    // management for pause process

    //! Request to pause the tracking module
    void request_pause();

    //! Check if the pause of the tracking module is requested or not
    bool pause_is_requested() const;

    //! Check if the tracking module is paused or not
    bool is_paused() const;

    //! Resume the tracking module
    void resume();

    //-----------------------------------------
    // variables

    //! config
    const std::shared_ptr<config> cfg_;

    //! camera model (equals to cfg_->camera_)
    camera::base* camera_;

    //! latest tracking state
    tracker_state_t tracking_state_ = tracker_state_t::NotInitialized;
    //! last tracking state
    tracker_state_t last_tracking_state_ = tracker_state_t::NotInitialized;

    //! current frame and its image
    data::frame curr_frm_;
    //! image of the current frame
    cv::Mat img_gray_;

    //! elapsed microseconds for each tracking
    double elapsed_ms_ = 0.0;

protected:
    //-----------------------------------------
    // tracking processes

    //! Main stream of the tracking module
    void track();

    //! Try to initialize with the current frame
    bool initialize();

    //! Track the current frame
    bool track_current_frame();

    //! Update the motion model using the current and last frames
    void update_motion_model();

    //! Replace the landmarks if the `replaced` member has the valid pointer
    void apply_landmark_replace();

    //! Update the camera pose of the last frame
    void update_last_frame();

    //! Optimize the camera pose of the current frame
    bool optimize_current_frame_with_local_map();

    //! Update the local map
    void update_local_map();

    //! Acquire more 2D-3D matches using initial camera pose estimation
    void search_local_landmarks();

    //! Check the new keyframe is needed or not
    bool new_keyframe_is_needed() const;

    //! Insert the new keyframe derived from the current frame
    void insert_new_keyframe();

    //! system
    system* system_ = nullptr;
    //! mapping module
    mapping_module* mapper_ = nullptr;
    //! global optimization module
    global_optimization_module* global_optimizer_ = nullptr;

    // ORB extractors
    //! ORB extractor for left/monocular image
    feature::orb_extractor* extractor_left_ = nullptr;
    //! ORB extractor for right image
    feature::orb_extractor* extractor_right_ = nullptr;
    //! ORB extractor only when used in initializing
    feature::orb_extractor* ini_extractor_left_ = nullptr;

    //! map_database
    data::map_database* map_db_ = nullptr;

    // Bag of Words
    //! BoW vocabulary
    data::bow_vocabulary* bow_vocab_ = nullptr;
    //! BoW database
    data::bow_database* bow_db_ = nullptr;

    //! initializer
    module::initializer initializer_;

    //! frame tracker for current frame
    const module::frame_tracker frame_tracker_;

    //! relocalizer
    module::relocalizer relocalizer_;

    //! pose optimizer
    const optimize::pose_optimizer pose_optimizer_;

    //! keyframe inserter
    module::keyframe_inserter keyfrm_inserter_;

    //! reference keyframe
    data::keyframe* ref_keyfrm_ = nullptr;
    //! local keyframes
    std::vector<data::keyframe*> local_keyfrms_;
    //! local landmarks
    std::vector<data::landmark*> local_landmarks_;

    //! the number of tracked keyframes in the current keyframe
    unsigned int num_tracked_lms_ = 0;

    //! last frame
    data::frame last_frm_;

    //! latest frame ID which succeeded in relocalization
    unsigned int last_reloc_frm_id_ = 0;

    //! motion model
    Mat44_t velocity_;
    //! motion model is valid or not
    bool velocity_is_valid_ = false;

    //! current camera pose from reference keyframe
    //! (to update last camera pose at the beginning of each tracking)
    Mat44_t last_cam_pose_from_ref_keyfrm_;

    //-----------------------------------------
    // mapping module status

    //! mutex for mapping module status
    mutable std::mutex mtx_mapping_;

    //! mapping module is enabled or not
    bool mapping_is_enabled_ = true;

    //-----------------------------------------
    // management for pause process

    //! mutex for pause process
    mutable std::mutex mtx_pause_;

    //! Check the request frame and pause the tracking module
    bool check_and_execute_pause();

    //! the tracking module is paused or not
    bool is_paused_ = false;

    //! Pause of the tracking module is requested or not
    bool pause_is_requested_ = false;
};

} // namespace openvslam

#endif // OPENVSLAM_TRACKING_MODULE_H
