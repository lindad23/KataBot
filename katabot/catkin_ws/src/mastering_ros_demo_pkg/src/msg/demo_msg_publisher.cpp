#include "ros/ros.h"
#include "mastering_ros_demo_pkg/demo_msg.h"
#include <iostream>
#include <sstream>

using demo_msg = mastering_ros_demo_pkg::demo_msg;

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_msg_publisher");
    ros::NodeHandle node_obj;
    auto number_publisher = node_obj.advertise<demo_msg>("/numbers", 10);
    ros::Rate loop_rate(10);
    int number_count = 0;
    
    while (ros::ok()) {
        demo_msg msg;
        msg.greeting = "hello world";
        msg.number = number_count;
        std::stringstream ss;
        ss << msg;
        ROS_INFO("%s", ss.str().c_str());
        // ROS_INFO("%d %s", msg.number, msg.greeting.c_str());
        number_publisher.publish(msg);
        loop_rate.sleep();
        ++number_count;
    }
    return 0;
}
