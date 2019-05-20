#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/camera/base.h"
#include "openvslam/track/tracker.h"
#include "openvslam/map/local_mapper.h"
#include "openvslam/map/loop_closer.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/camera_database.h"
#include "openvslam/data/map_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/util/converter.h"
#include "openvslam/publisher/map_publisher.h"
#include "openvslam/publisher/frame_publisher.h"

#include <thread>
#include <iomanip>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace openvslam {

system::system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path)
        : cfg_(cfg), camera_(cfg->camera_) {
    spdlog::debug("CONSTRUCT: system");

    std::cout << R"(  ___               __   _____ _      _   __  __  )" << std::endl;
    std::cout << R"( / _ \ _ __  ___ _ _\ \ / / __| |    /_\ |  \/  | )" << std::endl;
    std::cout << R"(| (_) | '_ \/ -_) ' \\ V /\__ \ |__ / _ \| |\/| | )" << std::endl;
    std::cout << R"( \___/| .__/\___|_||_|\_/ |___/____/_/ \_\_|  |_| )" << std::endl;
    std::cout << R"(      |_|                                         )" << std::endl;
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
    assert(map_db_);

    pause_other_threads();

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        // 1. frame statsを取得

        const auto frm_stats = map_db_->get_frame_statistics();

        // 2. フレームの姿勢を復元しながら保存

        const auto num_valid_frms = frm_stats.get_num_valid_frames();
        const auto reference_keyframes = frm_stats.get_reference_keyframes();
        const auto rel_cam_poses_from_ref_keyfrms = frm_stats.get_relative_cam_poses();
        const auto timestamps = frm_stats.get_timestamps();
        const auto is_lost_frms = frm_stats.get_lost_frames();

        if (num_valid_frms == 0) {
            spdlog::warn("there are no valid frames, cannot dump frame trajectory");
            resume_other_threads();
            return;
        }

        std::ofstream ofs(path, std::ios::out);
        if (!ofs.is_open()) {
            spdlog::critical("cannot create a file at {}", path);
            throw std::runtime_error("cannot create a file at " + path);
        }

        spdlog::info("dump frame trajectory in \"{}\" format from frame {} to frame {} ({} frames)",
                     format, reference_keyframes.begin()->first, reference_keyframes.rbegin()->first, num_valid_frms);

        const auto rk_itr_bgn = reference_keyframes.begin();
        const auto rc_itr_bgn = rel_cam_poses_from_ref_keyfrms.begin();
        const auto rk_itr_end = reference_keyframes.end();
        const auto rc_itr_end = rel_cam_poses_from_ref_keyfrms.end();
        auto rk_itr = rk_itr_bgn;
        auto rc_itr = rc_itr_bgn;

        int offset = rk_itr->first;
        unsigned int prev_frm_id = 0;
        for (unsigned int i = 0; i < num_valid_frms; ++i, ++rk_itr, ++rc_itr) {
            // frame IDを確認
            assert(rk_itr->first == rc_itr->first);
            const auto frm_id = rk_itr->first;

            // lost frameかどうかを確認
            if (is_lost_frms.at(frm_id)) {
                spdlog::warn("frame {} is lost", frm_id);
                continue;
            }

            // フレームがスキップされていないか確認
            if (frm_id != i + offset) {
                spdlog::warn("frame(s) from {} to {} is/are skipped", prev_frm_id + 1, frm_id - 1);
                offset = frm_id - i;
            }

            // reference keyframeのポインタと絶対姿勢
            auto ref_keyfrm = rk_itr->second;
            const Mat44_t ref_cam_pose_rw = ref_keyfrm->get_cam_pose();
            // reference keyframe -> frame の相対姿勢
            const Mat44_t rel_cam_pose_cr = rc_itr->second;

            // フレームの姿勢を計算
            const Mat44_t cam_pose_cw = rel_cam_pose_cr * ref_cam_pose_rw;
            const Mat44_t cam_pose_wc = cam_pose_cw.inverse();

            if (format == "KITTI") {
                ofs << std::setprecision(9)
                    << cam_pose_wc(0, 0) << " " << cam_pose_wc(0, 1) << " " << cam_pose_wc(0, 2) << " " << cam_pose_wc(0, 3) << " "
                    << cam_pose_wc(1, 0) << " " << cam_pose_wc(1, 1) << " " << cam_pose_wc(1, 2) << " " << cam_pose_wc(1, 3) << " "
                    << cam_pose_wc(2, 0) << " " << cam_pose_wc(2, 1) << " " << cam_pose_wc(2, 2) << " " << cam_pose_wc(2, 3) << std::endl;
            }
            else if (format == "TUM") {
                const Mat33_t& rot_wc = cam_pose_wc.block<3, 3>(0, 0);
                const Vec3_t& trans_wc = cam_pose_wc.block<3, 1>(0, 3);
                const Quat_t quat_wc = Quat_t(rot_wc);
                ofs << std::setprecision(15)
                    << timestamps.at(frm_id) << " "
                    << std::setprecision(9)
                    << trans_wc(0) << " " << trans_wc(1) << " " << trans_wc(2) << " "
                    << quat_wc.x() << " " << quat_wc.y() << " " << quat_wc.z() << " " << quat_wc.w() << std::endl;
            }
            else {
                throw std::runtime_error("Not implemented: trajectory format \"" + format + "\"");
            }

            prev_frm_id = frm_id;
        }
        if (rk_itr != rk_itr_end || rc_itr != rc_itr_end) {
            spdlog::error("the sizes of frame statistics are not matched");
        }

        ofs.close();
    }

    resume_other_threads();
}

void system::save_keyframe_trajectory(const std::string& path, const std::string& format) const {
    assert(map_db_);

    pause_other_threads();

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        // 1. keyframesを取得してID順にソート

        auto keyfrms = map_db_->get_all_keyframes();
        std::sort(keyfrms.begin(), keyfrms.end(), [&](data::keyframe* keyfrm_1, data::keyframe* keyfrm_2) {
            return *keyfrm_1 < *keyfrm_2;
        });

        // 2. 姿勢を変換して保存

        if (keyfrms.empty()) {
            spdlog::warn("there are no valid keyframes, cannot dump keyframe trajectory");
            resume_other_threads();
            return;
        }

        std::ofstream ofs(path, std::ios::out);
        if (!ofs.is_open()) {
            spdlog::critical("cannot create a file at {}", path);
            throw std::runtime_error("cannot create a file at " + path);
        }

        spdlog::info("dump keyframe trajectory in \"{}\" format from keyframe {} to keyframe {} ({} keyframes)",
                     format, (*keyfrms.begin())->id_, (*keyfrms.rbegin())->id_, keyfrms.size());

        for (const auto keyfrm : keyfrms) {
            // フレームの姿勢とタイムスタンプを取得
            const Mat44_t cam_pose_cw = keyfrm->get_cam_pose();
            const Mat44_t cam_pose_wc = cam_pose_cw.inverse();
            const auto timestamp = keyfrm->timestamp_;

            if (format == "KITTI") {
                ofs << std::setprecision(9)
                    << cam_pose_wc(0, 0) << " " << cam_pose_wc(0, 1) << " " << cam_pose_wc(0, 2) << " " << cam_pose_wc(0, 3) << " "
                    << cam_pose_wc(1, 0) << " " << cam_pose_wc(1, 1) << " " << cam_pose_wc(1, 2) << " " << cam_pose_wc(1, 3) << " "
                    << cam_pose_wc(2, 0) << " " << cam_pose_wc(2, 1) << " " << cam_pose_wc(2, 2) << " " << cam_pose_wc(2, 3) << std::endl;
            }
            else if (format == "TUM") {
                const Mat33_t& rot_wc = cam_pose_wc.block<3, 3>(0, 0);
                const Vec3_t& trans_wc = cam_pose_wc.block<3, 1>(0, 3);
                const Quat_t quat_wc = Quat_t(rot_wc);
                ofs << std::setprecision(15)
                    << timestamp << " "
                    << std::setprecision(9)
                    << trans_wc(0) << " " << trans_wc(1) << " " << trans_wc(2) << " "
                    << quat_wc.x() << " " << quat_wc.y() << " " << quat_wc.z() << " " << quat_wc.w() << std::endl;
            }
            else {
                throw std::runtime_error("Not implemented: trajectory format \"" + format + "\"");
            }
        }

        ofs.close();
    }

    resume_other_threads();
}

void system::load_message_pack(const std::string& path) {
    assert(map_db_ && bow_db_);

    pause_other_threads();

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        // 1. initialize database

        map_db_->clear();
        bow_db_->clear();

        // 2. load binary bytes

        std::ifstream ifs(path, std::ios::in | std::ios::binary);
        if (!ifs.is_open()) {
            spdlog::critical("cannot load the file at {}", path);
            throw std::runtime_error("cannot load the file at " + path);
        }

        spdlog::info("load the MessagePack file of database from {}", path);
        std::vector<uint8_t> msgpack;
        while (true) {
            uint8_t buffer;
            ifs.read(reinterpret_cast<char*>(&buffer), sizeof(uint8_t));
            if (ifs.eof()) {
                break;
            }
            msgpack.push_back(buffer);
        }
        ifs.close();

        // 3. parse into JSON

        const auto json = nlohmann::json::from_msgpack(msgpack);

        // 4. load database

        // load static variables
        data::frame::next_id_ = json.at("frame_next_id").get<unsigned int>();
        data::keyframe::next_id_ = json.at("keyframe_next_id").get<unsigned int>();
        data::landmark::next_id_ = json.at("landmark_next_id").get<unsigned int>();
        // load database
        const auto json_cameras = json.at("cameras");
        cam_db_->from_json(json_cameras);
        const auto json_keyfrms = json.at("keyframes");
        const auto json_landmarks = json.at("landmarks");
        map_db_->from_json(cam_db_, bow_vocab_, bow_db_, json_keyfrms, json_landmarks);
        const auto keyfrms = map_db_->get_all_keyframes();
        for (const auto keyfrm : keyfrms) {
            bow_db_->add_keyframe(keyfrm);
        }
    }

    resume_other_threads();
}

void system::save_message_pack(const std::string& path) {
    pause_other_threads();

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        const auto cameras = cam_db_->to_json();
        nlohmann::json keyfrms;
        nlohmann::json landmarks;
        map_db_->to_json(keyfrms, landmarks);

        nlohmann::json json{{"cameras", cameras},
                            {"keyframes", keyfrms},
                            {"landmarks", landmarks},
                            {"frame_next_id", static_cast<unsigned int>(data::frame::next_id_)},
                            {"keyframe_next_id", static_cast<unsigned int>(data::keyframe::next_id_)},
                            {"landmark_next_id", static_cast<unsigned int>(data::landmark::next_id_)}};

        std::ofstream ofs(path, std::ios::out | std::ios::binary);

        if (ofs.is_open()) {
            spdlog::info("save the MessagePack file of database to {}", path);
            const auto msgpack = nlohmann::json::to_msgpack(json);
            ofs.write(reinterpret_cast<const char*>(msgpack.data()), msgpack.size() * sizeof(uint8_t));
            ofs.close();
        }
        else {
            spdlog::critical("cannot create a file at {}", path);
        }
    }

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
