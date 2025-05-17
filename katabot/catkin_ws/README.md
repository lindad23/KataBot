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
```

