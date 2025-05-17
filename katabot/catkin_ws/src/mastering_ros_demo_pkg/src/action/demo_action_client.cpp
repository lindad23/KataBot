#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "mastering_ros_demo_pkg/demo_actionAction.h"
#include <iostream>

using demo_action = mastering_ros_demo_pkg::demo_actionAction;
using demo_action_goal = mastering_ros_demo_pkg::demo_actionGoal;
using demo_action_feedback_const_ptr = mastering_ros_demo_pkg::demo_actionFeedbackConstPtr;

demo_action_feedback_const_ptr last_feedback;
void feedback_callback(const demo_action_feedback_const_ptr& feedback){
    last_feedback = feedback;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_action_client");
    if (argc != 3) {
        ROS_INFO("%d", argc);
        ROS_WARN("Usage: demo_action_client <goal> <time_to_preempt_in_sec>");
        return 1;
    }
    actionlib::SimpleActionClient<demo_action> ac("demo_action_server", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    demo_action_goal goal;
    goal.count = atoi(argv[1]);
    ac.sendGoal(
        goal,
        actionlib::SimpleActionClient<demo_action>::SimpleDoneCallback(),
        actionlib::SimpleActionClient<demo_action>::SimpleActiveCallback(),
        &feedback_callback
    );
    ROS_INFO("Sending Goal [%d] and Preempt time of [%.2f]s", goal.count, atof(argv[2]));

    bool finished_before_timeout = ac.waitForResult(ros::Duration(atof(argv[2])));
    ac.cancelGoal();

    auto state = ac.getState();
    auto result = ac.getResult();
    if (finished_before_timeout) {
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before time out. Last feedback=%d", last_feedback->current_number);
    }
    return 0;
}
