#include "ros/ros.h"
#include "mastering_ros_demo_pkg/demo_srv.h"
#include <iostream>

using demo_srv = mastering_ros_demo_pkg::demo_srv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_service_client");
    ros::NodeHandle node_obj;
    ros::Rate loop_rate(10);
    auto client = node_obj.serviceClient<demo_srv>("demo_service");
    while (ros::ok()) {
        demo_srv srv;
        srv.request.in = "Sending from Here";
        if (client.call(srv)) {
            ROS_INFO("From Client [%s], Server says [%s]", srv.request.in.c_str(), srv.response.out.c_str());
        } else {
            ROS_ERROR("Failed to call service");
            return 1;
        }
        ros::spinOnce();  // If there are callback functions, spinOnce() will process them.
        loop_rate.sleep();
    }
    return 0;
}