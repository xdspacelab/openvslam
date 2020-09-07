#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


class pose_to_path_publisher
{
private:
    ros::NodeHandle nh;
    ros::Publisher path_pub;
    ros::Subscriber pose_sub;
    nav_msgs::Path path_msg;

public:
    void poseCallback(const geometry_msgs::PoseStamped msg)
    {
        path_msg.header.frame_id = "map";
        path_msg.poses.push_back(msg);
        path_pub.publish(path_msg);
    }

    pose_to_path_publisher()
    {
        path_pub = nh.advertise<nav_msgs::Path>("/openvslam/path", 1);
        pose_sub = nh.subscribe("/openvslam/camera_pose", 1, &pose_to_path_publisher::poseCallback, this);
    }
    
};

////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
    // Initialize ROS node
    ros::init(argc, argv, "path_publisher");
    ros::start();

    pose_to_path_publisher node;
    ROS_INFO("path_publisher is initialized.");
    ros::spin();
    ros::shutdown();

  return 0;
}