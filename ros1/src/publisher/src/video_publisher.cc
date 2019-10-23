#include <ros/ros.h>

#include <iostream>
#include <popl.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "video_publisher");

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
    const ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    const image_transport::Publisher publisher = it.advertise("/video/image_raw", 1);

    cv::Mat frame;
    cv::VideoCapture video;
    sensor_msgs::ImagePtr msg;

    // load video file
    if (!video.open(video_file_path->value(), cv::CAP_FFMPEG)) {
        std::cerr << "can't load video file" << std::endl;
        std::cerr << std::endl;
        return EXIT_FAILURE;
    }

    ros::Rate pub_rate(video.get(cv::CAP_PROP_FPS));

    while (nh.ok() && video.read(frame)) {
        // send message
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        publisher.publish(msg);
        ros::spinOnce();
        pub_rate.sleep();
    }
    return EXIT_SUCCESS;
}