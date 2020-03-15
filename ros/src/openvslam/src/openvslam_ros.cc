#include <openvslam_ros.h>

#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace openvslam_ros {
system::system(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : SLAM_(cfg, vocab_file_path), cfg_(cfg), it_(image_transport::ImageTransport(nh_)), tp_0_(std::chrono::steady_clock::now()),
      mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE)) {}

mono::mono(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path) {
    sub_ = it_.subscribe("camera/image_raw", 1, &mono::callback, this);
}
void mono::callback(const sensor_msgs::ImageConstPtr& msg) {
    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto encoding = (cfg_->camera_->color_order_ == openvslam::camera::color_order_t::Gray) ? "mono8" : "bgr8";
    SLAM_.feed_monocular_frame(cv_bridge::toCvShare(msg, encoding)->image, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
}

stereo::stereo(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path,
               const bool rectify)
    : system(cfg, vocab_file_path, mask_img_path),
      rectifier_(rectify ? std::make_shared<openvslam::util::stereo_rectifier>(cfg) : nullptr),
      left_sf_(image_transport::SubscriberFilter(it_, "/camera/left/image_raw", 1)),
      right_sf_(image_transport::SubscriberFilter(it_, "/camera/right/image_raw", 1)),
      sync_(message_filters::Synchronizer(SyncPolicy(10), left_sf_, right_sf_)) {
    sync_.registerCallback(boost::bind<void>(&stereo::callback, this, _1, _2));
}

void stereo::callback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right) {
    auto encoding = (cfg_->camera_->color_order_ == openvslam::camera::color_order_t::Gray) ? "mono8" : "bgr8";
    auto leftcv = cv_bridge::toCvShare(left, encoding)->image;
    auto rightcv = cv_bridge::toCvShare(right, encoding)->image;
    if (leftcv.empty() || rightcv.empty()) {
        return;
    }

    if (rectifier_) {
        rectifier_->rectify(leftcv, rightcv, leftcv, rightcv);
    }

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    SLAM_.feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
}
} // namespace openvslam_ros
