#ifndef OPENVSLAM_SYSTEM_H
#define OPENVSLAM_SYSTEM_H

#include "openvslam/type.h"
#include "openvslam/data/bow_vocabulary.h"

#include <string>
#include <thread>
#include <mutex>
#include <atomic>

#include <opencv2/core/core.hpp>

namespace openvslam {

class config;

namespace camera {
class base;
} // namespace camera

namespace data {
class camera_database;
class map_database;
class bow_database;
} // namespace data

namespace map {
class local_mapper;
class loop_closer;
} // namespace map

namespace track {
class tracker;
} // namespace track

namespace publisher {
class map_publisher;
class frame_publisher;
} // namespace publisher

class system {
public:
    system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path);

    ~system();

    void startup(const bool need_initialize = true);

    Mat44_t track_for_monocular(const cv::Mat& img, const double timestamp, const cv::Mat& mask = cv::Mat{});

    Mat44_t track_for_stereo(const cv::Mat& left_img, const cv::Mat& right_img, const double timestamp, const cv::Mat& mask = cv::Mat{});

    Mat44_t track_for_RGBD(const cv::Mat& rgb_img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask = cv::Mat{});

    const std::shared_ptr<publisher::map_publisher> get_map_publisher() const {
        return map_publisher_;
    }

    const std::shared_ptr<publisher::frame_publisher> get_frame_publisher() const {
        return frame_publisher_;
    }

    void activate_mapping_module();

    void deactivate_mapping_module();

    bool get_mapping_module_status() const;

    void activate_loop_detector();

    void deactivate_loop_detector();

    bool get_loop_detector_status() const;

    bool loop_BA_is_running() const;

    void abort_loop_BA();

    void pause_tracker();

    bool tracker_is_paused() const;

    void resume_tracker();

    void request_reset();

    bool reset_is_requested() const;

    void request_terminate();

    bool terminate_is_requested() const;

    void shutdown();

    void save_frame_trajectory(const std::string& path, const std::string& format) const;

    void save_keyframe_trajectory(const std::string& path, const std::string& format) const;

    void load_message_pack(const std::string& path);

    void save_message_pack(const std::string& path);

private:
    //! check reset request
    void check_reset_request();

    /**
     * Pause local mapper and loop closer threads
     */
    void pause_other_threads() const;

    /**
     * Resume local mapper and loop closer threads
     */
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
    track::tracker* tracker_ = nullptr;

    //! local mapper
    map::local_mapper* local_mapper_ = nullptr;
    //! local mapper thread
    std::unique_ptr<std::thread> local_mapper_thread_ = nullptr;

    //! loop closer
    map::loop_closer* loop_closer_ = nullptr;
    //! loop closer thread
    std::unique_ptr<std::thread> loop_closer_thread_ = nullptr;

    //! frame publisher
    std::shared_ptr<publisher::frame_publisher> frame_publisher_ = nullptr;
    //! map publisher
    std::shared_ptr<publisher::map_publisher> map_publisher_ = nullptr;

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
