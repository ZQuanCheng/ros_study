#include "ros/ros.h"

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "hello_cpp_node");

    ROS_INFO("ROS Hello World!!!! by cpp");

    return 0;
}