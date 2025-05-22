# ROS official packages
## Controller Manager工作原理
ROS1和ROS2的Controller有所不同，在ROS1中更麻烦，需要在URDF文件中对每个待驱动关节通过`transmission`标签给出`joint`和`hardwareInterface`的关系，也就是对`joint`的`Interface`类型进行一次声明，也就是硬件接口，而真机的硬件或者仿真的硬件信息也必须指定`joint`来发送接口信息，`hardwareInterface`有如下几种类型，前三种是写在URDF的joint中声明用的，后面两种类型是在Controller Manager中传输用的：

1. EffortJointInterface：发送effort命令
2. VelocityJointInterface：发送Velocity命令
3. PositionJointInterface：发送Position命令
---
4. JointCommandInterfaces：向硬件驱动发送命令
5. JointStateInterfaces：硬件驱动向Controller Manger发送状态

# Moveit!
为了使Moveit!能够控制真机/仿真，我们需要两个控制器：
1. moveit controller manager，只需完成关节轨迹的配置文件`src/moveit_generated_pkg/config/simple_moveit_controllers.yaml`，并和下面启动的JointTrajectoryController参数路径对应即可
2. ros controller，需要启动`position_controllers/JointTrajectoryController`控制器，在moveit中的`simple_moveit_controller_manager.launch.xml`会自动启动`config/ros_controllers.yaml`，所以我们把配置写在这里就好了

这里总的控制流程就是：moveit controller manager -> ros controller -> hardwareInterface (Real/Gazebo)

**注意**：
1. `Failed to validate trajectory: couldn't receive full current joint state within 1s`：Moveit!无法读取到机器人的joint_states，由于默认的`joint_state_controller`会发布`joint_states`到`/seven_dof_arm/joint_states`下（controller manager有一个命名空间），因此我们需要将`move_group.launch`中的`move_group` node加入[节点映射，请见代码](./src/moveit_generated_pkg/launch/move_group.launch#104)

## 配置Moveit控制器
只需要自定义配置文件`[robot_name]_moveit_controller_manager.launch.xml`（这里`robot_name`是`six_dof_arm`），该launch文件的加载流程如下，其设置的param会在move_group node中自动被加载：
```bash
move_group.launch
  └── includes trajectory_execution.launch.xml
         └── includes six_dof_arm_moveit_controller_manager.launch.xml
                 └── sets param moveit_controller_manager
```

