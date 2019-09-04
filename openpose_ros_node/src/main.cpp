#include <iostream>
#include <yaml-cpp/yaml.h>

//-ROS-//
#include <ros/ros.h>
#include <ros/package.h>
#include <openpose_ros_node/openpose_ros.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openpose_ros_2d");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    std::string path = ros::package::getPath("openpose_ros_node");
    path.append("/config/config.yaml");
    YAML::Node config_2d = YAML::LoadFile(path);
    openpose_node pose_2d(config_2d);
    pose_2d.init_openpose();

    while(ros::ok())
    {
        ros::spinOnce();
    }
    std::cout<<"Done";
    return 0;
}