#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <popl.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto video_file_path = op.add<popl::Value<std::string>>("m", "video", "video file path");

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
    if (!video_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // initialize this node
    auto node = std::make_shared<rclcpp::Node>("video_publisher");
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;

    const image_transport::Publisher publisher = image_transport::create_publisher(node.get(), "/video/image_raw", custom_qos);

    cv::Mat frame;
    cv::VideoCapture video;
    sensor_msgs::msg::Image::SharedPtr msg;

    // load video file
    if (!video.open(video_file_path->value(), cv::CAP_FFMPEG)) {
        std::cerr << "can't load the video file" << std::endl;
        std::cerr << std::endl;
        return EXIT_FAILURE;
    }

    rclcpp::WallRate pub_rate(video.get(cv::CAP_PROP_FPS));
    rclcpp::executors::SingleThreadedExecutor exec;

    exec.add_node(node);

    while (rclcpp::ok() && video.read(frame)) {
        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher.publish(msg);
        exec.spin_some();
        pub_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
