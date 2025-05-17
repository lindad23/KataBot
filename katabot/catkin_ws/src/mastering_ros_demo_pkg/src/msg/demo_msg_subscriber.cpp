#include "ros/ros.h"
#include "mastering_ros_demo_pkg/demo_msg.h"
#include <iostream>

using demo_msg = mastering_ros_demo_pkg::demo_msg;

void number_callback(const demo_msg::ConstPtr& msg) {
    ROS_INFO("Received greeting [%s]", msg->greeting.c_str());
    ROS_INFO("Received number [%d]", msg->number);
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_msg_subscriber");
    ros::NodeHandle node_obj;
    auto number_subscriber = node_obj.subscribe("/numbers", 10, number_callback);
    ros::spin();
    return 0;
}