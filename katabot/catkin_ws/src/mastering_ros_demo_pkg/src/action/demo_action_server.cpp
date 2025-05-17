#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "mastering_ros_demo_pkg/demo_actionAction.h"
#include <iostream>

class DemoAction {
    using demo_action = mastering_ros_demo_pkg::demo_actionAction;
    using demo_action_feedback = mastering_ros_demo_pkg::demo_actionFeedback;
    using demo_action_result = mastering_ros_demo_pkg::demo_actionResult;
    using demo_action_goal_const_ptr = mastering_ros_demo_pkg::demo_actionGoalConstPtr;

protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<demo_action> as;
    demo_action_feedback feedback;
    demo_action_result result;

    std::string action_name;
    int goal, progress, progress_rate;

public:
    DemoAction(std::string name, int rate):
        as(nh, name, boost::bind(&DemoAction::execute_callback, this, _1), false),
        action_name(name), progress_rate(rate) {
            as.registerPreemptCallback(boost::bind(&DemoAction::preempt_callback, this));
            as.start();
        }

    void preempt_callback() {
        ROS_WARN("%s got preempted!", action_name.c_str());
        result.final_count = progress;
        as.setPreempted(result, "I got Preempted");
    }

    void execute_callback(const demo_action_goal_const_ptr &goal) {
        if (!as.isActive() || as.isPreemptRequested()) return;
        ros::Rate rate(progress_rate);
        ROS_INFO("%s is precessing the goal %d", action_name.c_str(), goal->count);
        for (progress = 1; progress <= goal->count; progress++) {
            if (!ros::ok()) {
                result.final_count = progress;
                as.setAborted(result, "I failed!");
                ROS_INFO("%s Shutting down", action_name.c_str());
                break;
            }
            if (!as.isActive() || as.isPreemptRequested()) return;
            if (goal->count <= progress) {
                result.final_count = progress;
                as.setSucceeded(result);
                ROS_INFO("%s Succeeded at getting to goal %d", action_name.c_str(), goal->count);
            } else {
                feedback.current_number = progress;
                as.publishFeedback(feedback);
                ROS_INFO("Setting to goal %d / %d", feedback.current_number, goal->count);
            }
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    if (argc != 2) {
        ROS_INFO("Get argc=%d", argc);
        ROS_WARN("Usage: demo_action_server <progress_rate>");
        return 1;
    }
    ros::init(argc, argv, "demo_action_server");
    ROS_INFO("Starting Demo Action Server");
    DemoAction demo_action_obj(ros::this_node::getName(), atoi(argv[1]));
    ros::spin();
    return 0;
}
