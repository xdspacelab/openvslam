#include <ros/ros.h>

#include <util/image_util.h>

#include <iostream>
#include <popl.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_publisher");

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto img_dir_path = op.add<popl::Value<std::string>>("i", "img-dir", "directory path which contains images");
    auto img_fps = op.add<popl::Value<unsigned int>>("", "fps", "FPS of images", 30);

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
    if (!img_dir_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // initialize this node
    const ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher publisher = it.advertise("/video/image_raw", 1);

    sensor_msgs::ImagePtr msg;

    // load video file
    image_sequence sequence(img_dir_path->value(), img_fps->value());
    const auto frames = sequence.get_frames();

    ros::Rate pub_rate(img_fps->value());

    for (unsigned int i = 0; i < frames.size() && nh.ok(); ++i) {
        const auto& frame = frames.at(i);
        std::cout << "next img: " << frame.img_path_ << std::endl;
        const auto img = cv::imread(frame.img_path_, cv::IMREAD_UNCHANGED);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        publisher.publish(msg);
        ros::spinOnce();
        pub_rate.sleep();
    }
    return EXIT_SUCCESS;
}