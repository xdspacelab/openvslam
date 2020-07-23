#ifndef OPENVSLAM_SYSTEM_H
#define OPENVSLAM_SYSTEM_H

#include "openvslam/type.h"
#include "openvslam/data/bow_vocabulary.h"

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

#include <opencv2/core/core.hpp>

namespace openvslam {

class config;
class tracking_module;
class mapping_module;
class global_optimization_module;

namespace camera {
class base;
} // namespace camera

namespace data {
class camera_database;
class map_database;
class bow_database;
} // namespace data

namespace publish {
class map_publisher;
class frame_publisher;
} // namespace publish

class system {
public:
    //! Constructor
    system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path);

    //! Destructor
    ~system();

    //-----------------------------------------
    // system startup and shutdown

    //! Startup the SLAM system
    void startup(const bool need_initialize = true);

    //! Shutdown the SLAM system
    void shutdown();

    //-----------------------------------------
    // data I/O

    //! Save the frame trajectory in the specified format
    void save_frame_trajectory(const std::string& path, const std::string& format) const;

    //! Save the keyframe trajectory in the specified format
    void save_keyframe_trajectory(const std::string& path, const std::string& format) const;

    //! Load the map database from the MessagePack file
    void load_map_database(const std::string& path) const;

    //! Save the map database to the MessagePack file
    void save_map_database(const std::string& path) const;

    //! Get the map publisher
    const std::shared_ptr<publish::map_publisher> get_map_publisher() const;

    //! Get the frame publisher
    const std::shared_ptr<publish::frame_publisher> get_frame_publisher() const;

    //-----------------------------------------
    // module management

    //! Enable the mapping module
    void enable_mapping_module();

    //! Disable the mapping module
    void disable_mapping_module();

    //! The mapping module is enabled or not
    bool mapping_module_is_enabled() const;

    //! Enable the loop detector
    void enable_loop_detector();

    //! Disable the loop detector
    void disable_loop_detector();

    //! The loop detector is enabled or not
    bool loop_detector_is_enabled() const;

    //! Loop BA is running or not
    bool loop_BA_is_running() const;

    //! Abort the loop BA externally
    void abort_loop_BA();

    //-----------------------------------------
    // data feeding methods

    //! Feed a monocular frame to SLAM system
    //! (NOTE: distorted images are acceptable if calibrated)
    Mat44_t feed_monocular_frame(const cv::Mat& img, const double timestamp, const cv::Mat& mask = cv::Mat{});

    //! Feed a stereo frame to SLAM system
    //! (Note: Left and Right images must be stereo-rectified)
    Mat44_t feed_stereo_frame(const cv::Mat& left_img, const cv::Mat& right_img, const double timestamp, const cv::Mat& mask = cv::Mat{});

    //! Feed an RGBD frame to SLAM system
    //! (Note: RGB and Depth images must be aligned)
    Mat44_t feed_RGBD_frame(const cv::Mat& rgb_img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask = cv::Mat{});

    //-----------------------------------------
    // management for pause

    //! Pause the tracking module
    void pause_tracker();

    //! The tracking module is paused or not
    bool tracker_is_paused() const;

    //! Resume the tracking module
    void resume_tracker();

    //-----------------------------------------
    // management for reset

    //! Request to reset the system
    void request_reset();

    //! Reset of the system is requested or not
    bool reset_is_requested() const;

    //-----------------------------------------
    // management for terminate

    //! Request to terminate the system
    void request_terminate();

    //!! Termination of the system is requested or not
    bool terminate_is_requested() const;

private:
    //! Check reset request of the system
    void check_reset_request();

    //! Pause the mapping module and the global optimization module
    void pause_other_threads() const;

    //! Resume the mapping module and the global optimization module
    void resume_other_threads() const;

    //! config
    const std::shared_ptr<config> cfg_;
    //! camera model
    camera::base* camera_ = nullptr;

    //! camera database
    data::camera_database* cam_db_ = nullptr;

    //! map database
    data::map_database* map_db_ = nullptr;

    //! BoW vocabulary
    data::bow_vocabulary* bow_vocab_ = nullptr;

    //! BoW database
    data::bow_database* bow_db_ = nullptr;

    //! tracker
    tracking_module* tracker_ = nullptr;

    //! mapping module
    mapping_module* mapper_ = nullptr;
    //! mapping thread
    std::unique_ptr<std::thread> mapping_thread_ = nullptr;

    //! global optimization module
    global_optimization_module* global_optimizer_ = nullptr;
    //! global optimization thread
    std::unique_ptr<std::thread> global_optimization_thread_ = nullptr;

    //! frame publisher
    std::shared_ptr<publish::frame_publisher> frame_publisher_ = nullptr;
    //! map publisher
    std::shared_ptr<publish::map_publisher> map_publisher_ = nullptr;

    //! system running status flag
    std::atomic<bool> system_is_running_{false};

    //! mutex for reset flag
    mutable std::mutex mtx_reset_;
    //! reset flag
    bool reset_is_requested_ = false;

    //! mutex for terminate flag
    mutable std::mutex mtx_terminate_;
    //! terminate flag
    bool terminate_is_requested_ = false;

    //! mutex for flags of enable/disable mapping module
    mutable std::mutex mtx_mapping_;

    //! mutex for flags of enable/disable loop detector
    mutable std::mutex mtx_loop_detector_;
};

} // namespace openvslam

#endif // OPENVSLAM_SYSTEM_H
