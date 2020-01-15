#include <openvslam/system.h>
#include <openvslam/config.h>
#include "openvslam/publish/map_publisher.h"

#include <iostream>
#include <chrono>
#include <numeric>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

void mono_tracking(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path,
                   const std::string& mask_img_path, const bool eval_log, const std::string& map_db_path) {
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();

    std::vector<double> track_times;
    const auto tp_0 = std::chrono::steady_clock::now();

    // initialize this node
    auto node = std::make_shared<rclcpp::Node>("run_slam");
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;


    // publisher
    auto odometry_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);

    // run the SLAM as subscriber
    image_transport::Subscriber sub = image_transport::create_subscription(
        node.get(), "camera/image_raw", [&](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            const auto tp_1 = std::chrono::steady_clock::now();
            const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0).count();

            // input the current frame and estimate the camera pose
            SLAM.feed_monocular_frame(cv_bridge::toCvShare(msg, "bgr8")->image, timestamp, mask);

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);
        },
        "raw", custom_qos);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    std::shared_ptr<openvslam::publish::map_publisher> map_publisher_ = SLAM.get_map_publisher();

    rclcpp::WallRate pub_rate(10);
    auto cam_pose_ = map_publisher_->get_current_cam_pose();

    while (rclcpp::ok()) {
        // SLAM get the motion matrix publisher
        cam_pose_ = map_publisher_->get_current_cam_pose();

        // Extract rotation matrix and translation vector from 
        Eigen::Matrix3d rotation_matrix = cam_pose_.block(0, 0, 3, 3);
        Eigen::Vector3d translation_vector = cam_pose_.block(0, 3, 3, 1);

        tf2::Matrix3x3 tf_rotation_matrix(rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                          rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                          rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));

        tf2::Vector3 tf_translation_vector(translation_vector(0), translation_vector(1), translation_vector(2));

        tf2::Transform transform_tf(tf_rotation_matrix, tf_translation_vector);

        // Create odometry message and update it with current camera pose
        nav_msgs::msg::Odometry odom_msg_;
        
        odom_msg_.header.stamp = node->now();
        odom_msg_.header.frame_id = "camera_frame";
        odom_msg_.child_frame_id = "base_link_frame";

        odom_msg_.pose.pose.orientation.x = transform_tf.getRotation().getX();
        odom_msg_.pose.pose.orientation.y = transform_tf.getRotation().getY();
        odom_msg_.pose.pose.orientation.z = transform_tf.getRotation().getZ();
        odom_msg_.pose.pose.orientation.w = transform_tf.getRotation().getW();

        odom_msg_.pose.pose.position.x = transform_tf.getOrigin().getX();
        odom_msg_.pose.pose.position.y = transform_tf.getOrigin().getY();
        odom_msg_.pose.pose.position.z = transform_tf.getOrigin().getZ();

        odometry_pub_->publish(odom_msg_);
        exec.spin_some();
        pub_rate.sleep();
    }

    // shutdown the SLAM process
    SLAM.shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    if (track_times.size()) {
        std::sort(track_times.begin(), track_times.end());
        const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
        std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
        std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
    }
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif
    rclcpp::init(argc, argv);

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("", "map-db", "store a map database at this path after SLAM", "");

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(setting_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        mono_tracking(cfg, vocab_file_path->value(), mask_img_path->value(), eval_log->is_set(), map_db_path->value());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}