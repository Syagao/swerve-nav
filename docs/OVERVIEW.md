# 项目概览

当前源码基准和验证状态见 [PROGRESS.md](PROGRESS.md)。

## 用户目标

建立单车移动机器人仿真项目，系统学习建图、定位、规划、控制和自主导航。机器人采用指定开源四轮独立转向底盘的 URDF/Xacro；优先复用 Nav2、SLAM Toolbox、ros2_control、ros_gz。重点深入运动学解算和轨迹跟踪，后续能够替换并比较自己的算法，通过日志、曲线和仿真结果验证效果。

目标顺序为：模型与传感器 → 底盘控制与运动学 → 里程计与 TF → SLAM → 已知地图定位 → 规划与跟踪 → 自主导航。这是学习目标，不是自动执行计划；每次推进的具体步骤由用户命令确定。

## 环境与语言约束

| 项目 | 用户指定内容 |
| --- | --- |
| 宿主与子系统 | Windows 11 + WSL2 |
| Linux | Ubuntu 26.04 LTS |
| ROS | ROS 2 Lyrical |
| 仿真器 | Gazebo Jetty / Gazebo Sim 10 |
| GPU | NVIDIA RTX 4060 Laptop GPU |
| 图形工具 | 后续用户已提供 Gazebo 模型截图并确认直接命令可驱动车辆；此前 RViz/关节 GUI 无法恢复的问题仍未单独确认解决，见 [ISSUES.md](ISSUES.md) 的 Q-010 |
| 开发语言 | 本项目自有 C++ 代码统一采用 C++20，包含独立算法库；见 [D-004](DECISIONS.md) |

本机 ROS 2 Lyrical 的 ament 默认声明 `cxx_std_20`。[当前 CMakeLists.txt](../src/swerve_nav/CMakeLists.txt) 要求 C++20、关闭编译器扩展，并构建 `swerve_kinematics` 静态库。

用户于 2026-11-09 重申以 C++20 为主，随后要求将原协作参考与 Karpathy 准则直接整合到 [AGENTS.md](../AGENTS.md)。该文件是可独立使用的项目协作与编码规范入口，无需保留或另外读取两份源文件。

## 当前目录与边界

```text
agv_sim_ws/
├── AGENTS.md                         # 协作与编码约定
├── docs/                             # 项目知识库
├── src/
│   └── swerve_nav/                    # 当前唯一自有 ROS 包
│       ├── CMakeLists.txt / package.xml
│       ├── LICENSE                   # Gazebo launch 参考来源及 MIT 许可
│       ├── urdf/                     # 机器人、轮组、相机、雷达 Xacro；另有 sphere.urdf
│       ├── launch/                   # display.launch.py、gazebo.launch.py
│       ├── config/controllers.yaml   # 关节控制器配置
│       ├── include/swerve_kinematics.hpp  # 轮组类型、逆解与周期控制接口
│       └── src/swerve_kinematics.cpp      # 逆解及周期控制辅助函数
├── build/                            # colcon 生成的构建目录
├── install/                          # colcon 生成的工作区环境脚本与安装目录
├── log/                              # colcon 生成的日志目录
├── ref/                              # 九份参考资料，默认只读
└── .agents/skills/                    # 本地技能
```

包内尚无自有 world 文件，仿真使用 Gazebo 自带世界。构建限定 `--base-paths src`，避免扫描 `ref/` 中的参考包。历史进展见 [PROGRESS.md](PROGRESS.md)。

参考目录历史上使用过 `REF/` 和 `参考项目/`，当前为 `ref/`。[.gitignore](../.gitignore) 忽略参考副本，参考代码与自有源码分开维护。

## 参考资料基准

2026-11-09 只读核对以下本地快照。用户要求继续参考这些项目；新增的五个项目保留，未授权删除其他参考。下表用途用于选择阅读入口，不表示已经集成、构建通过或完成 Lyrical/Jetty 兼容性验证。

| 项目与本地入口 | 基准提交 | 本项目中的参考用途 |
| --- | --- | --- |
| [omnidirectional_four_wheeled_robot](../ref/omnidirectional_four_wheeled_robot/README.md) | `b40ae465683c` | 原始 URDF/Xacro 来源、最小仿真、关节控制和里程计链路 |
| [VehicleMotionControl](../ref/VehicleMotionControl/VehicleMotionControl/README.md) | `2290738898e6`，有本地修改 | 用户既有全向控制、有限舵角选解、轮组目标生成与轨迹跟踪思路 |
| [linorobot2](../ref/linorobot2/README.md) | `41d6b4e6b1f9`，分支 `lyrical` | 里程计融合、SLAM、Nav2 的系统集成；其差速/麦克纳姆解算不直接适用于本车 |
| [zinger_swerve_controller](../ref/zinger_swerve_controller/README.md) | `eaf2becbcf45` | 单轮转向模块建模、正逆解与运动曲线；需补本车机械角度限位约束 |
| [navigation2](../ref/navigation2/README.md) | `ae23216fd4d0` | 导航架构、规划/控制插件与接口 |
| [ros2_control](../ref/ros2_control/README.md) | `de472cd59309` | 控制管理器、硬件接口、状态与命令接口、关节限位 |
| [ompl](../ref/ompl/README.md) | `1ae6ac0bf7e9` | 采样运动规划与状态空间的后续研究 |
| [acados](../ref/acados/README.md) | `503364817c87` | 最优控制和 MPC 求解的后续研究 |
| [autoware_universe](../ref/autoware_universe/README.md) | `62a5739d4d5a` | 规划、控制及模块接口的工程参考，需核对道路车辆模型与本车的差异 |

除 VehicleMotionControl 外，各参考仓库在核对时均无工作区修改；新增五个项目均为 detached HEAD。基准记录只描述本地状态；参考代码的语言和依赖不改变本项目自有 C++20 约定。

### 原始模型来源

- 上游：[kshibata-g1412049/omnidirectional_four_wheeled_robot](https://github.com/kshibata-g1412049/omnidirectional_four_wheeled_robot)。
- 本地入口：[参考 README](../ref/omnidirectional_four_wheeled_robot/README.md)。
- 阅读基准：`b40ae465683c4d2889c45e07936f658bd364a02a`，提交时间 `2026-06-29T05:28:27+09:00`；初始化时工作树无修改。
- 本地 [package.xml](../ref/omnidirectional_four_wheeled_robot/package.xml) 声明包版本 `2.0.0`，构建类型 `ament_cmake`；这是已经迁移到 ROS 2 的参考实现。
- [LICENSE](../ref/omnidirectional_four_wheeled_robot/LICENSE)：MIT，版权归 Koji Shibata；后续复用时保留许可与来源。

本文记录的参考版本不等于远端最新版本；本次未拉取或修改上游。

### 用户既有运控库

- 入口：[VehicleMotionControl README](../ref/VehicleMotionControl/VehicleMotionControl/README.md)。用户明确只参考其中适用于全向底盘的控制器和解算方法。
- 2026-10-13 查阅时 HEAD 为 `2290738898e62dcf13a11204d3735c44a4a22512`，存在本地未提交修改；当次结论以实际文件为准，不能只凭该提交重现全部所读内容。
- 这是 C# / `netstandard2.0` 库；本项目仍采用 C++20。已完成静态阅读，尚未移植或在本项目运行其测试。可参考部分及轮组结构差异见 [ARCHITECTURE.md](ARCHITECTURE.md)。
