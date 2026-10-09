# 项目进度

更新日期：2026-10-09。源码基准：`200fc25`。

## 当前阶段

机器人模型与 RViz 显示阶段。自有资源包和显示入口已建立并构建通过；当前阻塞是 RViz 与关节 GUI 窗口无法从任务栏恢复，根因与恢复结果待确认，见 [Q-010](ISSUES.md#q-010两个-gui-只有任务栏图标无法恢复待解决)。Gazebo 动力学及后续导航尚未验收。

## 当前已完成

- `src/swerve_nav` 已包含模型、传感器 Xacro、显示 launch 及资源安装配置；模型职责见 [ARCHITECTURE.md](ARCHITECTURE.md)，当前坐标与实验基线见 [INTERFACES.md](INTERFACES.md)。
- 已按用户要求创建本地提交 `200fc25`（`feat: refine swerve robot model and add RViz display`），包含 7 个模型、启动与包配置文件；该次操作未推送远程。
- 模型修改阶段已通过 Xacro 展开、`check_urdf`、`gz sdf -p`、独立目录的 colcon 构建，以及现有 display launch 加载与 robot_description 参数检查。
- 已进行全转向几何间隙、落地高度、正定惯量及组合惯量、关节与传感器转换检查；这些是静态与数值校验，不是 Gazebo 运动试验。临时检查未作为仓库自动化测试提交。
- 2026-10-09 用户终端日志再次确认 `swerve_nav` 构建成功，三个显示节点启动、GUI 收到机器人描述；窗口可见性仍受 Q-010 阻塞。Ctrl+C 后 GUI `exit code -2` 的原因已确认，见 Q-009。

本次知识库同步只核对当前源码、Git 状态和已有验证证据，没有重跑构建、启动仿真或确认图形问题已恢复。更新前工作区无未提交变更。

构建及显示入口（在项目根目录执行）：

```bash
source /opt/ros/lyrical/setup.bash
colcon build --base-paths src --packages-select swerve_nav --symlink-install
source install/setup.bash
ros2 launch swerve_nav display.launch.py
```

上述环境加载只作用于执行它的终端；模型修改后需重启 display launch 才会重新展开 Xacro。

## 已有知识库

- 确定本项目根目录为 `/home/syagao/agv_sim_ws`，区别于 REF 内独立参考仓库。
- 读取初始化技能、Karpathy 编码准则，以及参考项目的 README、构建文件、模型、启动、控制配置、算法与测试脚本。
- 建立根 AGENTS.md 与 docs 索引、概览、架构、接口、决定、问题和进度文档。
- 按实际源码填充参考实现，并标记用户约束、源码事实、推导与未决事项。

## 当前未完成或未验证

- 两个 GUI 窗口恢复及当前模型交互显示验收。
- 自有 Gazebo world/启动入口、控制器配置、传感器桥接与实际数据验证；URDF 中的相关声明尚未构成完整可运行仿真。
- 底盘运动学、闭环控制、里程计、SLAM、定位与 Nav2 集成；质量、摩擦、阻尼和驱动约束的运动实验标定。
- 自有 C++ 编译目标与测试。当前构建通过的是资源包，不代表上述算法已实现。

## 历史边界

初始化记录“src 为空、无自有 ROS 包、根目录未识别出 Git 仓库”，对应当时空工作区构建 `Summary: 0 packages finished` 的结果。后续 `a95777b`、`200fc25` 及现存源码证明该状态已推进；这里保留旧结论的适用范围，避免与当前进度混淆。

此前助手在用户明确要求逐步控制实施前创建过仿真代码与文档，用户已经删除。那些文件和对应试运行结果不作为当前工程成果，不应恢复，也不据此标记某个开发阶段完成。当前状态以用户后续指令和现存文件为准。

## 后续入口

等待用户指定下一步。可查阅 [待确认事项](ISSUES.md) 和 [未决事项](DECISIONS.md)，但不自行选择任务实施。后续每次只记录当次授权范围、实际改动、验证证据与剩余问题。
