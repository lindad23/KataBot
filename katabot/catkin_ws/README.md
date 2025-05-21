# 《精通ROS机器人编程》用例笔记

| # | Pkgs | Info |
| - | - | - |
| 1 | [mastering_ros_demo_pkg](./src/mastering_ros_demo_pkg/) | (Caption 2) 入门demo, topic, service, action, customed msg/srv/action |
| 2 | [mastering_ros_robot_description_pkg](./src/mastering_ros_robot_description_pkg/) | (Caption 3) 介绍URDF和xacro使用方法，加入diff_wheeled和seven_dof_arm两种机器人 |
| 3 | [gazebo_demo_pkg](./src/gazebo_demo_pkg/) | (Caption 4) gazebo仿真环境中使用上述两种机器人，并使用Controller Manager对仿真的hardwareInterface接口进行控制，[Controller Manager工作原理理解请见notes.md](./notes.md#controller-manager工作原理)

# 使用方法
```bash
# Once
cd katabot/catkin_ws/src
catkin_init_workspace

# Build if update
cd katabot/catkin_ws
catkin_build
source devel/setup.zsh
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
### diff wheeled
```bash
# diff mobile robot view Launch
roslaunch mastering_ros_robot_description_pkg view_mobile_robot.launch
```

## 3 gazebo_demo_pkg
### seven dof arm
```bash
# Gazebo加载机械臂
roslaunch gazebo_demo_pkg seven_dof_arm_world.launch
# Gazebo加载机械臂, 并在上方生成一个相机
roslaunch gazebo_demo_pkg seven_dof_arm_with_rgbd_world.launch
rosrun image_view image_view image:=/rgbd_camera/depth/image_raw  # 新界面查看图像信息
rviz
# Gazebo加载机械臂, 并加载Controller Manager配置文件, 可对每个关节进行位控
roslaunch gazebo_demo_pkg gazebo_demo_pkg_control.launch
rostopic list  # 可以看到/seven_dof_arm/joint*_position_controller/command
rostopic pub /seven_dof_arm/joint4_position_controller/command std_msgs/Float64 "1.0"  # 向其发送指令
```

### diff wheeled
```bash
# 安装 teleop-twist-keyboard, teleop-twist-joy 用来控制小车
sudo apt-get install ros-noetic-teleop-twist-keyboard
sudo apt install ros-noetic-teleop-twist-joy
```
启动joystick控制的launch文件写在了[joystick.launch](./src/gazebo_demo_pkg/launch/diff_wheeled/joystick.launch)，分别启动joy_node和teleop_joy_node，并将对应的配置文件写在[xbox_joystick.yaml](./src/gazebo_demo_pkg/config/diff_wheeled/xbox_joystick.yaml)中

这里有两个控制小车的版本：

第一个是用`libgazebo_ros_diff_drive`直接控制小车（[URDF中插件使用方法](./src/mastering_ros_robot_description_pkg/urdf/diff_wheeled_robot.xacro#198)），无需Controller Manager，但是和真机控制不一致，但是启动简单，直接对`/cmd_vel`发送twist指令即可
```bash
roslaunch gazebo_demo_pkg diff_wheeled_gazebo.launch use_keyboard:=true use_joystick:=true
```

第二个是用`gazebo_ros_controller`仿真小车的硬件接口`hardwareInterface::VelocityJointInterface`（PID系数配置文件位于[diff_wheeled_gazebo_pid.yaml](./src/gazebo_demo_pkg/config/diff_wheeled/diff_wheeled_gazebo_pid.yaml)），再启动Controller Manager中的`DiffDriveController`来控制小车（需要注意，要将base_frame_id设置为`base_footprint`确保odom是映射到`base_footprint`上，[代码](./src/gazebo_demo_pkg/config/diff_wheeled/diff_wheeled_controller.yaml#15)），由于有命名空间和控制器，控制小车需对`/diff_wheeled/diff_drive_controller/cmd_vel`发送twist指令
```bash
roslaunch gazebo_demo_pkg diff_wheeled_gazebo_HW_control.launch use_keyboard:=true use_joystick:=true
```
