# 项目进度

更新日期：2026-11-24。源码基准：`5ae9d67` 加工作区未提交修改。

## 当前阶段

当前处于逆解与周期控制开发阶段。Gazebo 直接关节命令已有运行证据。纯 C++ 逆解与周期辅助函数已有函数体，`DriveControl` 已声明，周期编排和 ROS 接入待完成。用户自行编写、助手逐步指导的约定继续有效，见 [D-012](DECISIONS.md)。

`pick_target()` 已排在 `solve_ik()` 之前。当前算法源文件在 WSL 通过 `g++ -std=c++20 -Werror=return-type` 单文件编译。算法工况与 Gazebo 未重跑。

## 当前已完成

- 模型、显示与 Gazebo 启动入口、控制器 YAML、时钟桥接和依赖已建立；职责及接口分别见 [ARCHITECTURE.md](ARCHITECTURE.md)、[INTERFACES.md](INTERFACES.md)。
- `063a827` 加入仿真入口及关节控制，`dc2c3e4` 加入 C++20 静态库和转向轴速度计算，`5ae9d67` 包含 ±120° 转向限位、`fit_angle()` 和设计大纲。推送 `5ae9d67` 时已核对远程 `main`，当次构建成功，两个当时的占位函数有缺少返回值的警告。
- 此前无界面仿真实际验证了模型生成、三个控制器 active、8 个关节反馈及 `/clock` 推进；用户随后提供 Gazebo 截图，并于 2026-10-13 反馈直接关节命令可正常驱动车辆。尚无定量速度、停车误差或复杂工况验收结果。
- 2026-10-13 四个转向 joint1 改为 revolute，机械限位及 position 命令范围同步为 ±120°，滚动 joint2 保持 continuous。已通过 Xacro、check_urdf、SDF 转换和构建，并逐轮核对转换后的范围；未进行修改后运行中的限位测试。
- 当前工作区已有 `solve_ik()`、单轮候选选解与滞回、四轮统一限速，以及 `needs_stop()`、`brake()`、`align_scale()`、`track()` 的函数体。六个辅助函数集中在同一个匿名命名空间。接口和阶段转换见 [设计大纲](SWERVE_KINEMATICS_DESIGN.md)。
- 2026-11-06 曾在当时的工作区验证 1.00 m/s 配置。当前模型使用 0.80 m/s，轮半径 0.10 m 时对应 ±8 rad/s。提交 `5ae9d67` 时通过 Xacro 展开核对四轮转向关节和位置命令范围均为 ±120°。
- 2026-11-09 用户重申使用 C++20，并将原协作参考与 Karpathy 准则整合到 AGENTS.md。历史参考基准见 [OVERVIEW.md](OVERVIEW.md)。

包级 lint 和退出异常的验证限制见 [ISSUES.md](ISSUES.md) 的 Q-012、Q-013。

构建及当前 Gazebo 入口（在项目根目录执行）：

```bash
source /opt/ros/lyrical/setup.bash
colcon build --base-paths src --packages-select swerve_nav --symlink-install
source install/setup.bash
ros2 launch swerve_nav gazebo.launch.py
```

上述环境加载只作用于执行它的终端；模型修改后需停止旧仿真并重新启动。独立查看模型时使用 display.launch.py，两个入口的 joint_states 来源不要同时运行。

## 已有知识库

- 知识库最初建立在 `/home/syagao/agv_sim_ws` 工作区。当前检出的参考资料位于 `ref/`。
- 读取初始化技能、Karpathy 编码准则，以及参考项目的 README、构建文件、模型、启动、控制配置、算法与测试脚本。
- 建立根 AGENTS.md 与 docs 索引、概览、架构、接口、决定、问题和进度文档。
- 按实际源码填充参考实现，并标记用户约束、源码事实、推导与未决事项。

## 当前未完成或未验证

- `DriveControl` 构造函数和 `step()` 的定义、逆解与连续周期行为测试。当前尚无运动学 ROS 节点或 `kinematics.yaml`。
- `/cmd_vel` 接入、键盘控制整车、反馈参与的转向准备与命令超时停车。不得把现有两个关节控制器视为整车运动学控制器。
- 正解、里程计、SLAM、定位及 Nav2；自有 world、传感器系统与数据桥接也未完成。
- ±120° 限位的实际运行表现、横移/自转/组合运动，以及质量、摩擦和驱动约束的标定。
- RViz/关节 GUI 恢复结果仍待确认；支撑臂修改后的画面对比、桥接进程退出异常及包级 lint 见 [ISSUES.md](ISSUES.md)。

## 历史边界

初始化记录“src 为空、无自有 ROS 包、根目录未识别出 Git 仓库”，对应当时空工作区构建 `Summary: 0 packages finished` 的结果。后续 `a95777b`、`200fc25` 及现存源码证明该状态已推进；这里保留旧结论的适用范围，避免与当前进度混淆。

2026-10-09 的 `200fc25` 文档快照中，“只有模型显示、无 Gazebo 入口/控制器 YAML/自有 C++ 目标”是当时事实。随后 `063a827` 和 2026-10-13～14 的工作区修改提供了本页新增状态的依据；此前模型几何、惯量等静态检查不能替代后续动力学验收。

此前助手在用户明确要求逐步控制实施前创建过仿真代码与文档，用户已经删除。那些文件和对应试运行结果不作为当前工程成果，不应恢复，也不据此标记某个开发阶段完成。当前状态以用户后续指令和现存文件为准。

## 后续入口

下一步按 [设计大纲](SWERVE_KINEMATICS_DESIGN.md#实现顺序与验收) 验证整组逆解并实现 `DriveControl::step()`。ROS 节点接入安排在独立算法验证之后。
