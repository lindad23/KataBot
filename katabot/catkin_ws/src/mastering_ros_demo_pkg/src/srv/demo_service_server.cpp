#include "ros/ros.h"
#include "mastering_ros_demo_pkg/demo_srv.h"
#include <iostream>
#include <sstream>

using demo_srv = mastering_ros_demo_pkg::demo_srv;

bool demo_service_callback(demo_srv::Request &req, demo_srv::Response &res) {
    res.out = "Received Here";
    ROS_INFO("From Client [%s], Server says [%s]", req.in.c_str(), res.out.c_str());
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_service_server");
    ros::NodeHandle node_obj;
    auto service = node_obj.advertiseService("demo_service", demo_service_callback);
    ROS_INFO("Ready to receive from client.");
    ros::spin();
    return 0;
}
