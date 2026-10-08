# 参考接口与坐标

更新日期：2026-10-08。只描述参考源码与配置，不表示当前有节点或话题在运行。

## ROS 话题

| 话题 | 消息类型 | 发布者 → 使用者 / 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 外部命令 → controller_kinematics；使用 linear.x、linear.y、angular.z |
| `/position_controller/commands` | `std_msgs/msg/Float64MultiArray` | 逆解 → 转向控制器；四个角度，单位 rad |
| `/velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | 逆解 → 驱动控制器；四个角速度，单位 rad/s |
| `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_broadcaster → 状态发布器与里程计 |
| `/odom` | `nav_msgs/msg/Odometry` | odometry_publisher → 下游；frame_id=odom、child_frame_id=base_link |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 经桥接 → ROS 仿真时间 |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo 经桥接；模型指定 frame 为 body_link |
| `/camera/image` | `sensor_msgs/msg/Image` | Gazebo 经桥接 → 图像使用者 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo 经桥接 → 相机参数使用者 |

来源：[逆解](../REF/omnidirectional_four_wheeled_robot/src/controller_kinematics.cpp)、[里程计](../REF/omnidirectional_four_wheeled_robot/src/odometry_publisher.cpp)、[桥配置](../REF/omnidirectional_four_wheeled_robot/config/gz_bridge.yaml)。桥配置中的激活项均为 GZ_TO_ROS；其中 `/FrontLaser/scan` 映射被注释，模型中的前后雷达实例也被注释，不能假设当前默认有激光数据。

## 关节顺序与控制器

命令数组顺序固定为 `[FR, RR, FL, RL]`，应与 [controllers.yaml](../REF/omnidirectional_four_wheeled_robot/config/controllers.yaml) 一致。JointState 消息由里程计按关节名称查找，不按接收数组的位置直接假设轮序。

| 数组索引 | 参考前缀 | 转向关节 | 驱动关节 | 轮心相对机体中心的 (x, y)，m |
| --- | --- | --- | --- | --- |
| 0 | FR | FRwheel_joint1 | FRwheel_joint2 | (+0.375, +0.375) |
| 1 | RR | RRwheel_joint1 | RRwheel_joint2 | (+0.375, -0.375) |
| 2 | FL | FLwheel_joint1 | FLwheel_joint2 | (-0.375, +0.375) |
| 3 | RL | RLwheel_joint1 | RLwheel_joint2 | (-0.375, -0.375) |

轮半径为 0.2 m；joint1 绕局部 z 轴转向，joint2 绕局部 y 轴滚动。依据：[几何常量](../REF/omnidirectional_four_wheeled_robot/include/omnidirectional_four_wheeled_robot/robot_geometry.hpp)、[机器人 Xacro](../REF/omnidirectional_four_wheeled_robot/urdf/robot.urdf.xacro)、[轮组 Xacro](../REF/omnidirectional_four_wheeled_robot/urdf/wheel.urdf.xacro)。前缀与常用“前右/前左”方位不能直接对应，算法应先按源码坐标理解。

参考控制器类型为 `joint_state_broadcaster/JointStateBroadcaster`、`position_controllers/JointGroupPositionController`、`velocity_controllers/JointGroupVelocityController`。这是参考配置的事实，不代表已确认它们在目标版本中可直接使用。

## TF 与时间

- 参考里程计发布 `odom → base_link`；robot_state_publisher 根据 URDF 与 joint_states 发布机体内部 TF。
- URDF 的 `base_link → body_link` 为固定平移 `(0, 0, 0.3)` m，四轮转向关节挂在 body_link 下。
- 参考项目未提供 `map → odom` 的发布节点；SLAM/定位部分尚未接入。
- 参考 launch 为状态发布器、桥接、逆解、里程计、RViz 设置 `use_sim_time=true`；逆解实际使用 `create_wall_timer`，不能因此假设其定时器随仿真暂停。

未来命令是否使用带时间戳消息、是否增加 base_footprint、如何分配 TF 发布职责，均尚未作为本项目接口定案。
