#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/camera/base.h"
#include "openvslam/io/trajectory_io.h"
#include "openvslam/io/map_database_io.h"
#include "openvslam/track/tracker.h"
#include "openvslam/map/local_mapper.h"
#include "openvslam/map/loop_closer.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/camera_database.h"
#include "openvslam/data/map_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/publisher/map_publisher.h"
#include "openvslam/publisher/frame_publisher.h"

#include <thread>

#include <spdlog/spdlog.h>

namespace openvslam {

system::system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path)
        : cfg_(cfg), camera_(cfg->camera_) {
    spdlog::debug("CONSTRUCT: system");

    std::cout << R"(  ___               __   _____ _      _   __  __ )" << std::endl;
    std::cout << R"( / _ \ _ __  ___ _ _\ \ / / __| |    /_\ |  \/  |)" << std::endl;
    std::cout << R"(| (_) | '_ \/ -_) ' \\ V /\__ \ |__ / _ \| |\/| |)" << std::endl;
    std::cout << R"( \___/| .__/\___|_||_|\_/ |___/____/_/ \_\_|  |_|)" << std::endl;
    std::cout << R"(      |_|                                        )" << std::endl;
    std::cout << std::endl;
    std::cout << "Copyright (C) 2019," << std::endl;
    std::cout << "National Institute of Advanced Industrial Science and Technology (AIST)" << std::endl;
    std::cout << "All rights reserved." << std::endl;
    std::cout << std::endl;
    std::cout << "This is free software," << std::endl;
    std::cout << "and you are welcome to redistribute it under certain conditions." << std::endl;
    std::cout << "See the LICENSE file." << std::endl;
    std::cout << std::endl;

    // show configuration
    std::cout << *cfg_ << std::endl;

    // load ORB vocabulary
    spdlog::info("loading ORB vocabulary: {}", vocab_file_path);
#ifdef USE_DBOW2
    bow_vocab_ = new data::bow_vocabulary();
    try {
        bow_vocab_->loadFromBinaryFile(vocab_file_path);
    }
    catch (const std::exception& e) {
        spdlog::critical("wrong path to vocabulary");
        delete bow_vocab_; bow_vocab_ = nullptr;
        exit(EXIT_FAILURE);
    }
#else
    bow_vocab_ = new fbow::Vocabulary();
    bow_vocab_->readFromFile(vocab_file_path);
    if (!bow_vocab_->isValid()) {
        spdlog::critical("wrong path to vocabulary");
        delete bow_vocab_; bow_vocab_ = nullptr;
        exit(EXIT_FAILURE);
    }
#endif

    // database
    cam_db_ = new data::camera_database(camera_);
    map_db_ = new data::map_database();
    bow_db_ = new data::bow_database(bow_vocab_);

    // frame and map publisher
    frame_publisher_ = std::shared_ptr<publisher::frame_publisher>(new publisher::frame_publisher(cfg_, map_db_));
    map_publisher_ = std::shared_ptr<publisher::map_publisher>(new publisher::map_publisher(cfg_, map_db_));

    // tracker
    tracker_ = new track::tracker(cfg_, this, map_db_, bow_vocab_, bow_db_);
    // local mapper
    local_mapper_ = new map::local_mapper(map_db_, camera_->setup_type_ == camera::setup_type_t::Monocular);
    // loop closer
    loop_closer_ = new map::loop_closer(map_db_, bow_db_, bow_vocab_, camera_->setup_type_ != camera::setup_type_t::Monocular);

    // connect modules each other
    tracker_->set_local_mapper(local_mapper_);
    tracker_->set_loop_closer(loop_closer_);
    local_mapper_->set_tracker(tracker_);
    local_mapper_->set_loop_closer(loop_closer_);
    loop_closer_->set_tracker(tracker_);
    loop_closer_->set_local_mapper(local_mapper_);
}

system::~system() {
    loop_closer_thread_.reset(nullptr);
    delete loop_closer_; loop_closer_ = nullptr;

    local_mapper_thread_.reset(nullptr);
    delete local_mapper_; local_mapper_ = nullptr;

    delete tracker_; tracker_ = nullptr;

    delete bow_db_; bow_db_ = nullptr;
    delete map_db_; map_db_ = nullptr;
    delete cam_db_; cam_db_ = nullptr;
    delete bow_vocab_; bow_vocab_ = nullptr;

    spdlog::debug("DESTRUCT: system");
}

void system::startup(const bool need_initialize) {
    spdlog::info("startup SLAM system");
    system_is_running_ = true;

    if (!need_initialize) {
        tracker_->tracking_state_ = tracking_state_t::Lost;
    }

    local_mapper_thread_ = std::unique_ptr<std::thread>(new std::thread(&openvslam::map::local_mapper::run, local_mapper_));
    loop_closer_thread_ = std::unique_ptr<std::thread>(new std::thread(&openvslam::map::loop_closer::run, loop_closer_));
}

Mat44_t system::track_for_monocular(const cv::Mat& img, const double timestamp, const cv::Mat& mask) {
    assert(camera_->setup_type_ == camera::setup_type_t::Monocular);

    check_reset_request();

    const Mat44_t cam_pose_cw = tracker_->track_monocular_image(img, timestamp, mask);

    frame_publisher_->update(tracker_);
    if (tracker_->tracking_state_ == tracking_state_t::Tracking) {
        map_publisher_->set_current_cam_pose(cam_pose_cw);
    }

    return cam_pose_cw;
}

Mat44_t system::track_for_stereo(const cv::Mat& left_img, const cv::Mat& right_img, const double timestamp, const cv::Mat& mask) {
    assert(camera_->setup_type_ == camera::setup_type_t::Stereo);

    check_reset_request();

    const Mat44_t cam_pose_cw = tracker_->track_stereo_image(left_img, right_img, timestamp, mask);

    frame_publisher_->update(tracker_);
    if (tracker_->tracking_state_ == tracking_state_t::Tracking) {
        map_publisher_->set_current_cam_pose(cam_pose_cw);
    }

    return cam_pose_cw;
}

Mat44_t system::track_for_RGBD(const cv::Mat& rgb_img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask) {
    assert(camera_->setup_type_ == camera::setup_type_t::RGBD);

    check_reset_request();

    const Mat44_t cam_pose_cw = tracker_->track_RGBD_image(rgb_img, depthmap, timestamp, mask);

    frame_publisher_->update(tracker_);
    if (tracker_->tracking_state_ == tracking_state_t::Tracking) {
        map_publisher_->set_current_cam_pose(cam_pose_cw);
    }

    return cam_pose_cw;
}

void system::activate_mapping_module() {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    if (!system_is_running_) {
        spdlog::critical("please call system::activate_mapping_module() after system::startup()");
    }
    // local mapperを再開する
    local_mapper_->resume();
    // trackerに教える
    tracker_->set_mapping_module_status(true);
}

void system::deactivate_mapping_module() {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    if (!system_is_running_) {
        spdlog::critical("please call system::deactivate_mapping_module() after system::startup()");
    }
    // local mapperを止める
    local_mapper_->request_pause();
    // 止まるまで待つ
    while (!local_mapper_->is_paused()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    // trackerに教える
    tracker_->set_mapping_module_status(false);
}

bool system::get_mapping_module_status() const {
    return tracker_->get_mapping_module_status();
}

void system::activate_loop_detector() {
    std::lock_guard<std::mutex> lock(mtx_loop_detector_);
    loop_closer_->set_loop_detector_status(true);
}

void system::deactivate_loop_detector() {
    std::lock_guard<std::mutex> lock(mtx_loop_detector_);
    loop_closer_->set_loop_detector_status(false);
}

bool system::get_loop_detector_status() const {
    return loop_closer_->get_loop_detector_status();
}

bool system::loop_BA_is_running() const {
    return loop_closer_->loop_BA_is_running();
}

void system::abort_loop_BA() {
    loop_closer_->abort_loop_BA();
}

void system::pause_tracker() {
    tracker_->request_pause();
}

bool system::tracker_is_paused() const {
    return tracker_->is_paused();
}

void system::resume_tracker() {
    tracker_->resume();
}

void system::request_reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    reset_is_requested_ = true;
}

bool system::reset_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    return reset_is_requested_;
}

void system::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool system::terminate_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void system::shutdown() {
    // 他のスレッドで動いているものを止める
    local_mapper_->request_terminate();
    loop_closer_->request_terminate();
    // 止まるまで待つ
    while (!local_mapper_->is_terminated()
           || !loop_closer_->is_terminated()
           || loop_closer_->loop_BA_is_running()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    // スレッド終了まで待機
    local_mapper_thread_->join();
    loop_closer_thread_->join();

    spdlog::info("shutdown SLAM system");
    system_is_running_ = false;
}

void system::save_frame_trajectory(const std::string& path, const std::string& format) const {
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);
    trajectory_io.save_frame_trajectory(path, format);
    resume_other_threads();
}

void system::save_keyframe_trajectory(const std::string& path, const std::string& format) const {
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);
    trajectory_io.save_keyframe_trajectory(path, format);
    resume_other_threads();
}

void system::load_message_pack(const std::string& path) {
    pause_other_threads();
    io::map_database_io map_db_io(cam_db_, map_db_, bow_db_, bow_vocab_);
    map_db_io.load_message_pack(path);
    resume_other_threads();
}

void system::save_message_pack(const std::string& path) {
    pause_other_threads();
    io::map_database_io map_db_io(cam_db_, map_db_, bow_db_, bow_vocab_);
    map_db_io.save_message_pack(path);
    resume_other_threads();
}

void system::check_reset_request() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    if (reset_is_requested_) {
        tracker_->reset();
        reset_is_requested_ = false;
    }
}

void system::pause_other_threads() const {
    // local mapperを止める
    if (local_mapper_ && !local_mapper_->is_terminated()) {
        local_mapper_->request_pause();
        while (!local_mapper_->is_paused() && !local_mapper_->is_terminated()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    }
    // loop closerを止める
    if (loop_closer_ && !loop_closer_->is_terminated()) {
        loop_closer_->request_pause();
        while (!loop_closer_->is_paused() && !loop_closer_->is_terminated()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    }

}

void system::resume_other_threads() const {
    // loop closerを再開する
    if (loop_closer_) {
        loop_closer_->resume();
    }
    // local mapperを再開する
    if (local_mapper_) {
        local_mapper_->resume();
    }
}

} // namespace openvslam
