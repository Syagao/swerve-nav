# 项目与参考接口、坐标

当前源码基准见 [PROGRESS.md](PROGRESS.md)。参考项目的轮序、几何和接口按各自模型解读。

历史边界：`200fc25` 的八个运动关节均为 continuous，转向命令范围为 ±π，且当时没有自有控制器数组顺序。`063a827` 的 controllers.yaml 确定了轮序；2026-10-13 用户明确 ±120° 限位后，又同步修改了 URDF 关节类型和命令范围。以下描述当前定义。

## 当前显示接口与坐标

依据：[display.launch.py](../src/swerve_nav/launch/display.launch.py)、[机器人模型](../src/swerve_nav/urdf/robot.urdf.xacro)、[轮组模型](../src/swerve_nav/urdf/wheel.urdf.xacro)。以下为源码定义，实际验证范围见 [PROGRESS.md](PROGRESS.md)。

- 模型名 `swerve_robot`，包名 `swerve_nav`。显示链路使用 `/robot_description`、`/joint_states`、`/tf` 和 `/tf_static`；本项目尚无 `/odom`、`map → odom` 或 `odom → base_link` 发布节点。
- X 向前、Y 向左、Z 向上；长度 m、角度 rad、轮角速度 rad/s。四轮水平落地时，`base_link` 位于车体中心的地面投影；`base_link → body_link` 平移为 `(0, 0, 0.18)` m。
- 每个轮组为 `body_link → <前缀>_link1 → <前缀>_link2`：joint1 绕 Z 转向，为 `revolute`，URDF lower/upper 为 ±120°（±2π/3 rad）；joint2 绕局部 Y 滚动，保持 `continuous`。机械限位由模型定义，不能根据 GUI 滑块显示范围推断。

| 轮组前缀 | 转向关节 | 驱动关节 | 轮心相对 base_link 的 (x, y, z)，m |
| --- | --- | --- | --- |
| front_left | front_left_joint1 | front_left_joint2 | (+0.35, +0.31, 0.10) |
| front_right | front_right_joint1 | front_right_joint2 | (+0.35, -0.31, 0.10) |
| rear_left | rear_left_joint1 | rear_left_joint2 | (-0.35, +0.31, 0.10) |
| rear_right | rear_right_joint1 | rear_right_joint2 | (-0.35, -0.31, 0.10) |

转向枢轴离地 0.225 m，轮心位于其正下方，平面位置重合。每个轮组只有一个滚动轮，配独立转向关节；不是模块内左右双轮差速转向结构。

ros2_control 中 joint1 的 position 命令范围与机械限位共用 `steering_angle_limit`。URDF 声明转向角速度上限为 π rad/s，转向加减速上限共用 `steering_acceleration_limit`，值为 2π rad/s²。joint2 声明 velocity 命令。当前模型的滚动线速度上限为 1.0 m/s，滚动加速度和减速度上限均为 1.0 m/s²。按 0.10 m 半径换算，驱动命令范围为 ±10 rad/s，URDF 1.2 的驱动关节加速度和减速度上限均为 10 rad/s²。两种关节均声明 position/velocity 状态接口。验证状态见 [PROGRESS.md](PROGRESS.md)。

## 当前轮组配置类型

依据：[swerve_kinematics.hpp](../src/swerve_nav/include/swerve_kinematics.hpp) 的 `swerve_nav::WheelConfig`。这是静态配置聚合结构，已有十个 `double` 字段，没有自定义构造函数、配置校验或计算函数。

| 字段 | 单位 | 配置含义 |
| --- | --- | --- |
| `x`、`y` | m | 转向轴在 `base_link` 下的平面位置，当前模型中与轮心的平面位置重合 |
| `radius` | m | 滚动轮半径，用于接入 ROS 时换算线速度与驱动关节角速度 |
| `steer_min`、`steer_max` | rad | 有效转向角区间的下限与上限 |
| `drive_speed` | m/s | 滚动线速度幅值上限，同时适用于正转和反转 |
| `steer_speed` | rad/s | 转向角速度幅值上限，预留给舵角轨迹生成 |
| `steer_accel` | rad/s² | 转向加减速共用的角加速度幅值上限，预留给舵角轨迹生成 |
| `drive_accel` | m/s² | 驱动滚动速度幅值增加时的加速度上限 |
| `drive_decel` | m/s² | 驱动滚动速度幅值减小时的减速度上限 |

`WheelConfig` 各字段须由调用方填写。模型已声明驱动加减速限值，ROS 接入时需将角加速度按轮半径换算为线加速度，传给逐轮配置。`brake()` 和 `track()` 读取该配置。舵角速度和加速度字段目前未参与算法。

算法内部与 ROS 关节接口统一使用弧度。人工配置或日志可使用度，但应显式转换；当前头文件尚无度/弧度转换函数。轮组滚动线速度使用 m/s，驱动关节命令使用 rad/s，换算关系为 `omega = speed / radius`。

`BodyVelocity` 表示车体速度目标，`WheelState` 表示实测反馈，`WheelTarget` 表示舵角和滚动速度命令。四轮布局、反馈与命令按下方控制器轮序排列。`solve_ik()` 返回整组 `WheelGoal` 或无解。`DriveControl::step()` 已声明，`StepResult` 包含本周期命令和 `reachable`，后者仅表示整组逆解是否有解。完整接口见 [设计大纲](SWERVE_KINEMATICS_DESIGN.md#函数名与输入输出)。

## 当前 Gazebo 控制接口

依据：[controllers.yaml](../src/swerve_nav/config/controllers.yaml)、[gazebo.launch.py](../src/swerve_nav/launch/gazebo.launch.py)。控制管理器更新率为 100 Hz，配置使用仿真时间；这不表示外部命令必须按 100 Hz 发布。

| 话题 | 消息类型 | 当前用途 |
| --- | --- | --- |
| `/steering_controller/commands` | `std_msgs/msg/Float64MultiArray` | 四个转向角，单位 rad；ForwardCommandController 的 position 接口 |
| `/drive_controller/commands` | `std_msgs/msg/Float64MultiArray` | 四个滚动角速度，单位 rad/s；ForwardCommandController 的 velocity 接口 |
| `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_broadcaster 发布的实际关节反馈 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS 单向时间桥接，当前直接配置在 launch 中 |

两个命令数组都按 **左前、右前、左后、右后** 排列，分别对应上表四个 joint1 / joint2。读取 JointState 时按 `name` 对应 position/velocity，不假设反馈数组沿用命令轮序。

已提供的直行试验为四个转向角置零、四个驱动角速度设为 1 rad/s，理想无滑动时对应 0.1 m/s；停车使用四个零驱动速度。`ros2 topic pub --once` 只限定发布次数，当前转发控制器会保持最近命令，结束发布不等于停车。用户已反馈命令可正常驱动车辆，未记录定量速度或停车误差验收。

本项目尚无 `/cmd_vel` 订阅节点、运动学实现或自动命令超时处理。未来整车速度消息类型和超时参数尚未定案，不套用下方参考接口。

## 当前模型的实验基线

主要参数统一在 robot.urdf.xacro 顶部调整，尺寸与轮位不再沿用参考项目常量。

| 项目 | 当前基线及限制 |
| --- | --- |
| 车体 / 轮子 | 车体 0.60 × 0.40 × 0.16 m；轮半径 0.10 m、宽 0.05 m |
| 离地间隙 | 车体底部 0.10 m；转向叉最低处 0.09 m，两者不可混称 |
| 质量与惯量 | 总质量约 25.455 kg；车体含固定支撑的惯量采用等效长方体近似，转向叉采用三长方体组合，轮子采用圆柱 |
| 全转向包络 | 按轮胎旋转扫掠范围保守计算约 0.9062 × 0.8262 m；后续导航 footprint 不能只取车体尺寸，当前尚无 Nav2 footprint 配置 |

轮径参考 Jackal 的尺寸量级，布局参考 Ranger Mini 四轮独立转向；雷达和相机只借鉴 UST-10LX、D435 的外壳尺寸。资料链接已保留在 robot.urdf.xacro 参数注释中。这是简化实验模型，不是商业底盘或具体传感器的完整复现；质量、阻尼、摩擦和驱动约束仍是待运动实验标定的初始假设。

## 当前传感器定义

| 传感器 | Xacro 中 Gazebo topic 文本 | frame_id | 定义状态 |
| --- | --- | --- | --- |
| IMU | `imu` | `body_link` | 定义在 robot.urdf.xacro 中 |
| RGB 相机 | `camera/image` | `camera_optical_frame` | 已有外壳；仍是单 RGB 相机，不包含深度或双目功能 |
| 二维激光 | `lidar/scan` | `lidar_link` | 主模型已实例化；360°扫描定义，不是 UST-10LX 的真实视场模型 |

相机安装朝向与光学帧分离：`camera_link` 的 +X 向前，`camera_optical_frame` 的 +Z 向前、+X 向右、+Y 向下。`lidar_link` 原点是扫描中心，水平落地时离地 0.31 m，支架连接外壳与车顶。`front_marker_link` 仅作为车头方向的视觉标记。

这些是模型定义，display launch 不产生上述 Gazebo 传感器数据；当前 gazebo launch 也只配置了时钟桥接。传感器系统、数据产生及桥接验证留待对应阶段，未来自有雷达桥接应核对 `lidar/scan`，不能沿用参考项目被注释的 `/FrontLaser/scan`。

## 参考 ROS 话题

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

来源：[逆解](../ref/omnidirectional_four_wheeled_robot/src/controller_kinematics.cpp)、[里程计](../ref/omnidirectional_four_wheeled_robot/src/odometry_publisher.cpp)、[桥配置](../ref/omnidirectional_four_wheeled_robot/config/gz_bridge.yaml)。桥配置中的激活项均为 GZ_TO_ROS；其中 `/FrontLaser/scan` 映射被注释，模型中的前后雷达实例也被注释，不能假设当前默认有激光数据。

## 参考关节顺序与控制器

命令数组顺序固定为 `[FR, RR, FL, RL]`，应与 [controllers.yaml](../ref/omnidirectional_four_wheeled_robot/config/controllers.yaml) 一致。JointState 消息由里程计按关节名称查找，不按接收数组的位置直接假设轮序。

| 数组索引 | 参考前缀 | 转向关节 | 驱动关节 | 轮心相对机体中心的 (x, y)，m |
| --- | --- | --- | --- | --- |
| 0 | FR | FRwheel_joint1 | FRwheel_joint2 | (+0.375, +0.375) |
| 1 | RR | RRwheel_joint1 | RRwheel_joint2 | (+0.375, -0.375) |
| 2 | FL | FLwheel_joint1 | FLwheel_joint2 | (-0.375, +0.375) |
| 3 | RL | RLwheel_joint1 | RLwheel_joint2 | (-0.375, -0.375) |

轮半径为 0.2 m；joint1 绕局部 z 轴转向，joint2 绕局部 y 轴滚动。依据：[几何常量](../ref/omnidirectional_four_wheeled_robot/include/omnidirectional_four_wheeled_robot/robot_geometry.hpp)、[机器人 Xacro](../ref/omnidirectional_four_wheeled_robot/urdf/robot.urdf.xacro)、[轮组 Xacro](../ref/omnidirectional_four_wheeled_robot/urdf/wheel.urdf.xacro)。前缀与常用“前右/前左”方位不能直接对应，算法应先按源码坐标理解。

参考控制器类型为 `joint_state_broadcaster/JointStateBroadcaster`、`position_controllers/JointGroupPositionController`、`velocity_controllers/JointGroupVelocityController`。这是参考配置的事实，不代表已确认它们在目标版本中可直接使用。

## 参考 TF 与时间

- 参考里程计发布 `odom → base_link`；robot_state_publisher 根据 URDF 与 joint_states 发布机体内部 TF。
- URDF 的 `base_link → body_link` 为固定平移 `(0, 0, 0.3)` m，四轮转向关节挂在 body_link 下。
- 参考项目未提供 `map → odom` 的发布节点；SLAM/定位部分尚未接入。
- 参考 launch 为状态发布器、桥接、逆解、里程计、RViz 设置 `use_sim_time=true`；逆解实际使用 `create_wall_timer`，不能因此假设其定时器随仿真暂停。

未来命令是否使用带时间戳消息、是否增加 base_footprint、如何分配 TF 发布职责，均尚未作为本项目接口定案。
