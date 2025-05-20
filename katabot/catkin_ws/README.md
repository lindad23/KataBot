# 《精通ROS机器人编程》用例笔记

| # | Pkgs | Info |
| - | - | - |
| 1 | [mastering_ros_demo_pkg](./src/mastering_ros_demo_pkg/) | (Caption2) 入门demo, topic, service, action, customed msg/srv/action |

# 使用方法
```bash
# Once
cd katabot/catkin_ws/src
catkin_init_workspace

# Build if update
cd katabot/catkin_ws
catkin_build
```
## 1 mastering_ros_demo_pkg
```bash
# Topic
rosrun mastering_ros_demo_pkg demo_topic_publisher
rosrun mastering_ros_demo_pkg demo_topic_subscriber
# Message
rosrun mastering_ros_demo_pkg demo_msg_publisher
rosrun mastering_ros_demo_pkg demo_msg_subscriber
# Service
rosrun mastering_ros_demo_pkg demo_service_server
rosrun mastering_ros_demo_pkg demo_service_client
# Action
rosrun mastering_ros_demo_pkg demo_action_server 100
rosrun mastering_ros_demo_pkg demo_action_client 90 1.0
# Launch
roslaunch mastering_ros_demo_pkg demo_topic.launch
```

## 2 mastering_ros_robot_description_pkg
### pan tilt
```bash
# check urdf
sudo apt install liburdfdom-tools
# show urdf struct
check_urdf pan_tilt.urdf
urdf_to_graphiz pan_tilt.urdf
# pan_tilt launch
roslaunch mastering_ros_robot_description_pkg view_demo.launch
```
### seven dof arm
```bash
# 将xacro转为URDF
# In CMD
xacro seven_dof_arm.xacro > seven_dof_arm.urdf
# In Launch
<param name="robot_description" command="$(find xacro)/xacro $(find mastering_ros_robot_description_pkg)/urdf/seven_dof_arm.xacro" />

# arm view Launch
rosluanch mastering_ros_robot_description_pkg view_arm.launch
```
### diff mobile robot
```bash
# diff mobile robot view Launch
roslaunch mastering_ros_robot_description_pkg view_mobile_robot.launch
```

