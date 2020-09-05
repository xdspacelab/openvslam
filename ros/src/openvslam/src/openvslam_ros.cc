#include <openvslam_ros.h>

#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

void pose_odometry_pub(auto cam_pose_, auto pose_pub_){
    Eigen::Matrix3d rotation_matrix = cam_pose_.block(0, 0, 3, 3);
    Eigen::Vector3d translation_vector = cam_pose_.block(0, 3, 3, 1);

    tf2::Matrix3x3 tf_rotation_matrix(rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                      rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                      rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));
    
    tf2::Vector3 tf_translation_vector(translation_vector(0), translation_vector(1), translation_vector(2));

    tf_rotation_matrix = tf_rotation_matrix.inverse();
    tf_translation_vector = -(tf_rotation_matrix * tf_translation_vector);

    tf2::Transform transform_tf(tf_rotation_matrix, tf_translation_vector);

    tf2::Matrix3x3 rot_open_to_ros (0, 0, 1,
                                  -1, 0, 0,
                                   0,-1, 0);

    tf2::Transform transformA(rot_open_to_ros, tf2::Vector3(0.0, 0.0, 0.0));
    tf2::Transform transformB(rot_open_to_ros.inverse(), tf2::Vector3(0.0, 0.0, 0.0));

    transform_tf = transformA * transform_tf * transformB;

    ros::Time now = ros::Time::now();

    // Create pose message and update it with current camera pose
    geometry_msgs::PoseStamped camera_pose_msg_;
    camera_pose_msg_.header.stamp = now;
    camera_pose_msg_.header.frame_id = "map";
    camera_pose_msg_.pose.position.x = transform_tf.getOrigin().getX();
    camera_pose_msg_.pose.position.y = transform_tf.getOrigin().getY();
    camera_pose_msg_.pose.position.z = transform_tf.getOrigin().getZ();
    camera_pose_msg_.pose.orientation.x = transform_tf.getRotation().getX();
    camera_pose_msg_.pose.orientation.y = transform_tf.getRotation().getY();
    camera_pose_msg_.pose.orientation.z = transform_tf.getRotation().getZ();
    camera_pose_msg_.pose.orientation.w = transform_tf.getRotation().getW();
    pose_pub_.publish(camera_pose_msg_);

    // // transform broadcast
    // static tf2_ros::TransformBroadcaster tf_br;
   
    // geometry_msgs::TransformStamped transformStamped;

    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "map";
    // transformStamped.child_frame_id = "base_link_frame";
    // transformStamped.transform.translation.x = transform_tf.getOrigin().getX();
    // transformStamped.transform.translation.y = transform_tf.getOrigin().getY();
    // transformStamped.transform.translation.z = transform_tf.getOrigin().getZ();
    // transformStamped.transform.rotation.x = transform_tf.getRotation().getX();
    // transformStamped.transform.rotation.y = transform_tf.getRotation().getY();
    // transformStamped.transform.rotation.z = transform_tf.getRotation().getZ();
    // transformStamped.transform.rotation.w = transform_tf.getRotation().getW();

    // tf_br.sendTransform(transformStamped);
}

namespace openvslam_ros {
system::system(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : SLAM_(cfg, vocab_file_path), cfg_(cfg), it_(nh_), tp_0_(std::chrono::steady_clock::now()),
      mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE)) {}

mono::mono(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path) {
    sub_ = it_.subscribe("camera/image_raw", 1, &mono::callback, this);
    camera_pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("/openvslam/camera_pose", 1);
}

void mono::callback(const sensor_msgs::ImageConstPtr& msg) {
    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose = SLAM_.feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
    pose_odometry_pub(cam_pose, camera_pose_publisher);
}

stereo::stereo(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path,
               const bool rectify)
    : system(cfg, vocab_file_path, mask_img_path),
      rectifier_(rectify ? std::make_shared<openvslam::util::stereo_rectifier>(cfg) : nullptr),
      left_sf_(it_, "camera/left/image_raw", 1),
      right_sf_(it_, "camera/right/image_raw", 1),
      sync_(SyncPolicy(10), left_sf_, right_sf_) {
    sync_.registerCallback(&stereo::callback, this);
    camera_pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("/openvslam/camera_pose", 1);
}

void stereo::callback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right) {
    auto leftcv = cv_bridge::toCvShare(left)->image;
    auto rightcv = cv_bridge::toCvShare(right)->image;
    if (leftcv.empty() || rightcv.empty()) {
        return;
    }

    if (rectifier_) {
        rectifier_->rectify(leftcv, rightcv, leftcv, rightcv);
    }

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose = SLAM_.feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
    pose_odometry_pub(cam_pose, camera_pose_publisher);
}

rgbd::rgbd(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path),
      color_sf_(it_, "camera/color/image_raw", 1),
      depth_sf_(it_, "camera/depth/image_raw", 1),
      sync_(SyncPolicy(10), color_sf_, depth_sf_) {
    sync_.registerCallback(&rgbd::callback, this);
    camera_pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("/openvslam/camera_pose", 1);
}

void rgbd::callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth) {
    auto colorcv = cv_bridge::toCvShare(color)->image;
    auto depthcv = cv_bridge::toCvShare(depth)->image;
    if (colorcv.empty() || depthcv.empty()) {
        return;
    }

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose = SLAM_.feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);
    pose_odometry_pub(cam_pose, camera_pose_publisher);
}
} // namespace openvslam_ros