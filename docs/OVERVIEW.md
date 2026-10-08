# 项目概览

更新日期：2026-10-08。

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
| 图形工具 | 用户已说明 RViz 和 Gazebo 可正常运行 |
| 开发语言 | 本项目自有 C++ 代码统一采用 C++20，包含独立算法库；见 [D-004](DECISIONS.md) |

已只读确认：本机 `/opt/ros/lyrical/share/ament_cmake_ros_core/cmake/ament_ros_defaults.cmake` 声明 `cxx_std_20`。用户于 2026-10-08 明确要求本项目统一使用 C++20；后续自有构建目标按该标准配置。当前尚无自有 ROS 包或 C++ 构建目标，此项为已确定的开发约定，尚无对应编译验证结果。

## 当前目录与边界

```text
agv_sim_ws/
├── AGENTS.md                         # 协作与编码约定
├── docs/                             # 项目知识库
├── src/                              # 用户已创建，目前为空
├── build/                            # colcon 生成的构建目录
├── install/                          # colcon 生成的工作区环境脚本与安装目录
├── log/                              # colcon 生成的日志目录
├── REF/omnidirectional_four_wheeled_robot/  # 用户提供的参考仓库
├── .cursor/rules/karpathy-guidelines.mdc    # 用户指定编码准则
└── .agents/skills/                    # 本地技能
```

知识库初始化后，用户创建了 src，并明确要求初始化 ROS 2 工作区。已通过 `colcon build --base-paths src --symlink-install` 完成空工作区首次构建，生成 build/install/log；尚无自有 ROS 包。后续构建继续限定 `--base-paths src`，避免扫描 REF 中的参考包。

工作区根目录此前的 `git rev-parse --show-toplevel` 未识别出有效仓库；参考目录内部有独立 Git 仓库。本次没有初始化 Git。

## 参考资料基准

- 上游：[kshibata-g1412049/omnidirectional_four_wheeled_robot](https://github.com/kshibata-g1412049/omnidirectional_four_wheeled_robot)。
- 本地入口：[参考 README](../REF/omnidirectional_four_wheeled_robot/README.md)。
- 阅读基准：`b40ae465683c4d2889c45e07936f658bd364a02a`，提交时间 `2026-06-29T05:28:27+09:00`；初始化时工作树无修改。
- 本地 [package.xml](../REF/omnidirectional_four_wheeled_robot/package.xml) 声明包版本 `2.0.0`，构建类型 `ament_cmake`；这是已经迁移到 ROS 2 的参考实现。
- [LICENSE](../REF/omnidirectional_four_wheeled_robot/LICENSE)：MIT，版权归 Koji Shibata；后续复用时保留许可与来源。

本文记录的参考版本不等于远端最新版本；本次未拉取或修改上游。
