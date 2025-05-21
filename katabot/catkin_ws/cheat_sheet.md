# ROS速查表
## XML格式
常用的tag分为两种格式：
- `<tag attr="value" />`：自闭合标签（self-closing tag），所有信息都在**属性**中
- `<tag attr="value">content</tag>`：成对标签（start + end tag），既有**属性**也有**内部内容**`content`，`content`可以是其他**子标签**

## Launch脚本
**arg**：launch脚本内部参数，包含如下属性：
- `default`：可被覆盖赋值 `<arg name="paused" default="false"/>`，可在CLI中通过`paused:=true`来修改，或者在其他的调用的`<include>`标签中用`<arg name="paused" value="..."/>`来覆盖
- `value`：不可修改赋值 `<arg name="paused" value="false"/>`，外部传入的值无法对其进行修改
- 在脚本中可通过`$(arg paused)`来调用

**param**：ROS参数服务器键值对，包含如下属性
- `name`：参数名称
- `command`：命令行返回值赋值 `<param name="robot_description" command="$(find xacro)/xacro $(find mastering_ros_robot_description_pkg)/urdf/seven_dof_arm.xacro" />`
- `textfile`：文件读取赋值 `<param name="robot_description" textfile="$(find mastering_ros_robot_description_pkg)/urdf/pan_tilt.urdf" />`
- `value`：直接赋值 `<param name="max_speed" value="2.0"/>`
---
- `rosparam`读取`yaml`赋值：`<rosparam file="$(find my_pkg)/config/my_params.yaml" command="load">`
- 可被`rosparam`, `NodeHandle::getParma()`获取

**include**：启动另一个`launch`文件，包含如下属性：
- `file`：要启动的`launch`文件路径
---
包含子标签：
- `arg`：覆盖该`launch`文件的内部参数
```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="debug" value="$(arg debug)"/>
</include>
```

**node**：等价于用`rosrun`启动一个节点文件`src`或`script`，包含如下属性：
- `name, pkg, type`基础三个属性：名称，所属包，二进制/脚本程序名称`<node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"/>`
- `output`：控制输出显示位置，`output="screen"`或`"log"`，显示在屏幕（用于DEBUG）或日志文件（默认在`~/.ros/log/`）
- `respawn`：节点结束后是否自动重启，`respawn="false"`（默认为`"false"`）
- `respawn_delay`：设置重启延迟（单位：秒），`respawn_delay="5"`（默认为`"0.0"`）
- `required`：节点退出时是否终止整个`roslaunch`，`required="false"`（默认为`"false"`）
- `ns`：命名空间，把这个节点的所有名称（topic, service, param）都放到这个命名空间下
- `args`：CLI启动程序后加的参数
---
包含子标签：
- `remap`：`<remap from="原始名称" to="目标名称"/>`，支持对topic, service, action的发布/订阅映射

**group**：用于组织命名空间和条件控制，可以对内部的子标签`<node>, <include>, <param>`进行统一控制，包含如下属性：
- `if`：`<group if="$(use_keyboard)">`只有当条件为`true`时，执行内部内容（大小写敏感）
- `unless`：`<group unless="$(not_use_keyboard)">`只有当条件为`false`时，执行内部内容（大小写敏感）
- `ns`：`<group ns="robot1">`将该group内的所有产生的Node, Topic, Service, Param等都加入该命名空间下（路径前缀）

## 常用DEBUG可视化工具
### Topic
可以对node, topic进行可视化，易于理解调用关系
```bash
rosrun rqt_graph rqt_graph
```
### TF
查看由`robot_state_publisher`通过`joint_state`（topic）和`robot_description`（param）创建的TF关系树
```bash
rosrun rqt_tf_tree rqt_tf_tree
```