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

    tracking_module(const std::shared_ptr<config>& cfg, system* system, data::map_database* map_db,
                    data::bow_vocabulary* bow_vocab, data::bow_database* bow_db);

    ~tracking_module();

    void set_mapping_module(mapping_module* mapper);

    void set_global_optimization_module(global_optimization_module* global_optimizer);

    std::vector<cv::KeyPoint> get_initial_keypoints() const;

    std::vector<int> get_initial_matches() const;

    Mat44_t track_monocular_image(const cv::Mat& img, const double timestamp, const cv::Mat& mask = cv::Mat{});

    Mat44_t track_stereo_image(const cv::Mat& left_img_rect, const cv::Mat& right_img_rect, const double timestamp, const cv::Mat& mask = cv::Mat{});

    Mat44_t track_RGBD_image(const cv::Mat& img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask = cv::Mat{});

    //! reset databases
    void reset();

    //! set mapping module status
    void set_mapping_module_status(const bool mapping_is_enabled) {
        std::lock_guard<std::mutex> lock(mtx_mapping_);
        mapping_is_enabled_ = mapping_is_enabled;
    }

    //! get mapping module status
    bool get_mapping_module_status() const {
        std::lock_guard<std::mutex> lock(mtx_mapping_);
        return mapping_is_enabled_;
    }

    /**
     * Request to pause tracker
     */
    void request_pause();

    /**
     * Check if pause is requested or not
     * @return
     */
    bool pause_is_requested() const;

    /**
     * Check if tracker is paused or not
     * @return
     */
    bool is_paused() const;

    /**
     * Resume tracker
     */
    void resume();

    //! config
    const std::shared_ptr<config> cfg_;
    //! camera model (== cfg_->camera_)
    camera::base* camera_;

    //! tracking states
    tracker_state_t tracking_state_ = tracker_state_t::NotInitialized;
    tracker_state_t last_tracking_state_ = tracker_state_t::NotInitialized;

    //! current frame and its image
    data::frame curr_frm_;
    cv::Mat img_gray_;

    //! elapsed microseconds for each tracking
    double elapsed_ms_ = 0.0;

protected:
    // TODO: camera setupに応じてtrack関数を分ける
    void track();

    bool initialize();

    bool track_current_frame();

    bool localize_current_frame();

    void update_motion_model();

    void apply_landmark_replace();
    void update_last_frame();

    /**
     * Update local map and optimize the camera pose of current frame
     * @return
     */
    bool track_local_map();

    void update_local_map();
    void update_local_keyframes();
    void update_local_landmarks();

    void search_local_landmarks();

    /**
     * Check the new keyframe is needed or not
     * @return
     */
    bool new_keyframe_is_needed() const;

    /**
     * Insert the new keyframe derived from the current frame
     */
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

    //! current frameのインライア数
    unsigned int num_tracked_lms_ = 0;

    //! 前回のframe
    data::frame last_frm_;
    //! relocalizeに成功した時のframe ID
    unsigned int last_reloc_frm_id_ = 0;

    //! motion model
    bool velocity_is_valid_ = false;
    Mat44_t velocity_;

    //! current camera pose from reference keyframe
    //! (to update last camera pose at the beginning of each tracking)
    Mat44_t last_cam_pose_from_ref_keyfrm_;

    //-----------------------------------------
    //! mapping module status
    mutable std::mutex mtx_mapping_;
    bool mapping_is_enabled_ = true;

    //-----------------------------------------
    //! need mutex for access to pause procedure
    mutable std::mutex mtx_pause_;
    bool check_and_execute_pause();
    bool is_paused_ = false;
    bool pause_is_requested_ = false;
};

} // namespace openvslam

#endif // OPENVSLAM_TRACKING_MODULE_H
