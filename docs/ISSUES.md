# 待确认事项

更新日期：2026-11-09。当前问题与原参考源码疑点分开记录；未验证的建议不作为修复结论。表中的后续验证方式仅供用户授权相关步骤后使用。本次只同步资料路径和当前基准，没有复跑或关闭下列运行问题。

## 当前项目状态

- Q-002～Q-005 在自有模型中已作相应调整：使用明确方位前缀、转向叉与滚动轮分离碰撞体、校正圆柱惯量轴向、按落地状态定义 base_link。依据为 `200fc25` 的 [robot.urdf.xacro](../src/swerve_nav/urdf/robot.urdf.xacro)、[wheel.urdf.xacro](../src/swerve_nav/urdf/wheel.urdf.xacro)；当时仅通过几何与惯量检查。后续用户已确认直接命令可驱动车辆，仍不能据此认定接触参数和全部运动工况已验证。
- Q-007 的“未实例化雷达”不再适用于自有模型，当前已包含 lidar 宏及支架；这不代表传感器数据和桥接已打通，见 [当前传感器定义](INTERFACES.md#当前传感器定义)。
- Q-001 在自有最小仿真范围内已有运行证据，见 [PROGRESS.md](PROGRESS.md)，不扩大为传感器/导航栈完整兼容性结论。Q-006 所述参考逆解未移植；当前只有 WheelConfig 配置类型，运动仍依靠直接关节命令，超时停车和有限舵角选解尚未实现。下方原参考疑点保留其基准与依据，不表示修改过参考仓库。

## Q-009：Ctrl+C 后 GUI 的 exit code -2（原因已确认）

2026-10-09 用户提供的日志中，joint_state_publisher_gui 在 `user interrupted with ctrl-c (SIGINT)` 之后才报告 `exit code -2`。当次只读检查本机 Lyrical 安装源码确认：GUI 使用 `signal.signal(signal.SIGINT, signal.SIG_DFL)`，launch 的 `execute_local.py` 将非零退出状态记为 ERROR；`-2` 表示被 SIGINT 终止。[Python 退出码说明](https://docs.python.org/3/library/subprocess.html#subprocess.Popen.returncode)。

该记录可以由用户主动停止触发，不能据此判断启动失败。`Waiting for robot_description` 后出现 `Got description` 说明等待已结束；`Stereo is NOT SUPPORTED` 是立体显示能力提示，不阻止普通 RViz 三维显示。以上结论仅解释日志，不证明窗口已经可见或模型交互验收通过。本次未修改项目文件或上游退出处理。

## Q-010：两个 GUI 只有任务栏图标、无法恢复（待解决）

- **已确认现象**：2026-10-09 用户确认 RViz 与 joint_state_publisher_gui 两个窗口均无法从任务栏正常打开；启动日志显示模型初始化、GUI 收到机器人描述，且无所示渲染异常日志。
- **当次只读检查**：WSLg 已登记两个程序的窗口；X11 端均报告 `Normal`、`IsViewable`，RViz 窗口位于显示范围内。`glxinfo -B` 返回 `D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`、`Accelerated: yes`、OpenGL 4.6，说明该次探测的硬件加速可用。
- **结论边界**：根因尚未确认；上述结果不能证明 RViz 完整渲染链路正常，也不能将问题定性为 NVIDIA 驱动故障。启动和模型解析成功不能替代窗口可见性验证。
- **已提供但尚无结果的操作**：用 Alt+Tab 选中窗口后 Win+↑ 恢复；若无效，保存工作并停止 launch，在 Windows PowerShell 执行 `wsl --shutdown`，重新进入 Ubuntu 后加载 ROS 与工作区环境再启动。该命令会停止所有 WSL 发行版与其中进程，见 [微软命令说明](https://learn.microsoft.com/en-us/windows/wsl/basic-commands#shutdown)。未收到执行结果，不记录为已恢复。
- **验收条件**：两个窗口能够显示与恢复，RViz 能显示模型，关节滑块能够改变轮组姿态。
- **后续范围说明**：用户后来提供了 Gazebo 模型截图并确认小车可运动，说明独立 Gazebo 链路已能继续验证；没有收到 RViz 和关节 GUI 恢复的明确结果，Q-010 不标记为已解决，也不作为当前 Gazebo 学习的已确认阻塞。

## Q-011：支撑臂与车顶衔接出现三角斑块（已修改，画面复验待确认）

用户截图对应固定支撑臂顶面与车顶同处地面高度 0.260 m，存在共面深度冲突。`063a827` 在 robot.urdf.xacro 引入 0.002 m 的 `mount_arm_bearing_overlap`，使四个固定支撑臂的 visual/collision 同步下移 2 mm，嵌入固定轴承。

修改时已验证：支撑臂顶面为 0.258 m，与转向叉顶面仍有 3 mm 间隙；Xacro、check_urdf、SDF 转换和构建通过。未取得修改后的 GUI 画面对比，不宣称视觉复验已完成。

## Q-012：停止无界面仿真时桥接进程 exit code -11（待定位）

此前最小仿真运行检查中，模型生成、三个控制器 active、8 个关节反馈及时间推进均已确认；Ctrl+C 退出时 `ros_gz_bridge/parameter_bridge` 出现 -11（SIGSEGV）。该退出异常与 Q-009 的 SIGINT -2 不同，原因未定位，未作修复；本次知识库同步未复跑该退出过程。

## Q-013：包级 lint 尚未全部通过（已知检查限制）

此前包级检查发现 display.launch.py 的版权头和 flake8 格式问题。Gazebo launch 精简为 SPDX 短头后，已保留并安装完整 MIT 许可，但当前 ament_copyright 对该短头仍报告无法识别。未关闭检查或修改不相关文件来隐藏问题。

最近 CMake 修改已通过定向 ament_lint_cmake、清单解析和构建；这不代表整个包的测试通过。新增的空 C++ 文件尚未参加新一轮完整 lint，也没有运动学行为测试。

## 原参考源码疑点（保留初始化分析）

以下表格来自 2026-10-08 对参考基准的静态分析；当时未运行参考仿真。当前自有实现的处理状态以上文为准。

| 编号 | 已观察到的事实或推导 | 待确认方式 |
| --- | --- | --- |
| Q-001 版本适配 | 参考 README 与 CI 列出 Humble/Jazzy/Rolling，CI 未单列 Lyrical；控制器类型仍为 position_controllers / velocity_controllers | 在用户要求环境检查时核对目标发行版包、插件与 API，再分别验证构建和运行；不能用 CI 配置证明实际测试通过 |
| Q-002 轮组命名 | FR/RR/FL/RL 与实际 x/y 坐标的关系不同于常用方位缩写 | 按 INTERFACES 表核对 URDF、命令顺序与运动学，不凭名称猜测位置 |
| Q-003 重复轮组碰撞体 | 每组 link1 与 link2 都含同位置、同尺寸的圆柱 visual/collision | 检查实际接触与滚动受力，明确转向支架和滚动轮各自几何/质量；目前仅能确认定义重复 |
| Q-004 惯量与坐标 | 轮组几何和惯性坐标存在旋转，惯量公式的主轴需要核对 | 在相同坐标系下核算圆柱惯量和旋转；静态阅读不足以断言动力学正确 |
| Q-005 base_link 高度 | 轮心比 base_link 高 0.3 m，半径为 0.2 m；按水平落地几何推导，base_link 会在地面下约 0.1 m；里程计却固定输出 z=0 | 接入 TF/里程计前明确帧定义，比较模型姿态与所发布 TF；该数值是几何推导，不是本次运行测量 |
| Q-006 速度命令处理 | 逆解保存最后一次 Twist，未见命令超时清零；使用墙上时间定时器，未见节点内轮速饱和处理 | 分别检查停发命令、超限输入、仿真暂停/恢复以及执行器约束 |
| Q-007 激光未启用 | 模型中两处 laser_macro 实例和桥配置的激光条目被注释 | 用户要求传感器阶段时再确定传感器、topic/frame、频率与桥接配置 |
| Q-008 验证脚本适用范围 | smoke_test.sh 预期 Docker `/ros2_ws`，清理使用宽泛 pkill；轮速检查取所有关节速度最大值 | 不直接作为本工作区验收入口；另行确定隔离范围、按驱动关节的断言和真实位姿误差指标 |

证据入口：

- Q-001：[README](../参考项目/omnidirectional_four_wheeled_robot/README.md)、[CI](../参考项目/omnidirectional_four_wheeled_robot/.github/workflows/ci.yml)、[控制器配置](../参考项目/omnidirectional_four_wheeled_robot/config/controllers.yaml)。
- Q-002～Q-005：[模型](../参考项目/omnidirectional_four_wheeled_robot/urdf/robot.urdf.xacro)、[轮组](../参考项目/omnidirectional_four_wheeled_robot/urdf/wheel.urdf.xacro)、[几何常量](../参考项目/omnidirectional_four_wheeled_robot/include/omnidirectional_four_wheeled_robot/robot_geometry.hpp)、[里程计](../参考项目/omnidirectional_four_wheeled_robot/src/odometry_publisher.cpp)。
- Q-006：[逆解节点](../参考项目/omnidirectional_four_wheeled_robot/src/controller_kinematics.cpp)。
- Q-007：[模型](../参考项目/omnidirectional_four_wheeled_robot/urdf/robot.urdf.xacro)、[桥配置](../参考项目/omnidirectional_four_wheeled_robot/config/gz_bridge.yaml)。
- Q-008：[smoke_test.sh](../参考项目/omnidirectional_four_wheeled_robot/scripts/smoke_test.sh)。

问题的发现不授权修复。处理后记录实际验证结果，再改变状态。
