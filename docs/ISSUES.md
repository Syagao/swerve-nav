# 待确认事项

更新日期：2026-10-08。以下来自参考源码静态分析；本次没有执行仿真，也没有修改这些问题。表中的验证方式仅供用户授权相关步骤后使用。

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

- Q-001：[README](../REF/omnidirectional_four_wheeled_robot/README.md)、[CI](../REF/omnidirectional_four_wheeled_robot/.github/workflows/ci.yml)、[控制器配置](../REF/omnidirectional_four_wheeled_robot/config/controllers.yaml)。
- Q-002～Q-005：[模型](../REF/omnidirectional_four_wheeled_robot/urdf/robot.urdf.xacro)、[轮组](../REF/omnidirectional_four_wheeled_robot/urdf/wheel.urdf.xacro)、[几何常量](../REF/omnidirectional_four_wheeled_robot/include/omnidirectional_four_wheeled_robot/robot_geometry.hpp)、[里程计](../REF/omnidirectional_four_wheeled_robot/src/odometry_publisher.cpp)。
- Q-006：[逆解节点](../REF/omnidirectional_four_wheeled_robot/src/controller_kinematics.cpp)。
- Q-007：[模型](../REF/omnidirectional_four_wheeled_robot/urdf/robot.urdf.xacro)、[桥配置](../REF/omnidirectional_four_wheeled_robot/config/gz_bridge.yaml)。
- Q-008：[smoke_test.sh](../REF/omnidirectional_four_wheeled_robot/scripts/smoke_test.sh)。

问题的发现不授权修复。处理后记录实际验证结果，再改变状态。
