## Controller Manager工作原理
ROS1和ROS2的Controller有所不同，在ROS1中更麻烦，需要在URDF文件中对每个待驱动关节通过`transmission`标签给出`joint`和`hardwareInterface`的关系，也就是对`joint`的`Interface`类型进行一次声明，也就是硬件接口，而真机的硬件或者仿真的硬件信息也必须指定`joint`来发送接口信息，`hardwareInterface`有如下几种类型，前三种是写在URDF的joint中声明用的，后面两种类型是在Controller Manager中传输用的：

1. EffortJointInterface：发送effort命令
2. VelocityJointInterface：发送Velocity命令
3. PositionJointInterface：发送Position命令
---
4. JointCommandInterfaces：向硬件驱动发送命令
5. JointStateInterfaces：硬件驱动向Controller Manger发送状态


