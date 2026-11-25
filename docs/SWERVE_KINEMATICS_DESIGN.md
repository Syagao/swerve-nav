# 四轮独立转向逆运动学与周期目标生成

当前 C++20 算法源码已写出转向轴速度计算、整组逆解和周期控制辅助函数，`DriveControl` 构造函数和 `step()` 已声明，定义及 ROS 接入待完成。辅助函数的实现状态与当前编译检查见 [项目进度](PROGRESS.md)。本文记录接口及待实现的周期流程。依据见 [算法头文件](../src/swerve_nav/include/swerve_kinematics.hpp)、[算法源码](../src/swerve_nav/src/swerve_kinematics.cpp) 和 [构建配置](../src/swerve_nav/CMakeLists.txt)。

## 实车转向要求与当前模型

实车四个轮组的转向关节均要求 **±120° 机械限位**，即舵角范围为 `[−2π/3, +2π/3]` rad。转向零位对应轮组正向滚动时沿车体 +X。逆解和周期目标生成按这个有界位置坐标设计。

| 项目 | 当前源码定义 |
| --- | --- |
| 车体坐标 | `base_link` 的 X 向前、Y 向左、Z 向上，`wz` 逆时针为正。速度对应 `base_link` 原点。 |
| 轮序与轮位 | 左前、右前、左后、右后。转向轴的平面坐标依次为 `(+0.35, +0.31)`、`(+0.35, −0.31)`、`(−0.35, +0.31)`、`(−0.35, −0.31)` m。转向轴与轮心在平面上重合。 |
| 转向关节 `joint1` | 绕 +Z，URDF 类型为 `revolute`，声明速度上限 π rad/s、加减速上限均为 2π rad/s²。机械限位与 ros2_control 位置命令区间均为 `[−2π/3, +2π/3]` rad。 |
| 驱动关节 `joint2` | 绕轮组局部 +Y。轮半径 0.10 m，滚动线速度上限 1.0 m/s，加减速上限均为 1.0 m/s²。关节速度命令区间为 `[−10, +10]` rad/s，加减速上限均为 10 rad/s²。 |
| 执行方式 | 转向走 position 接口，驱动走 velocity 接口。控制管理器配置为 100 Hz 并使用仿真时间，生成器的 `dt` 由实际调用周期提供。 |

上表依据当前 [车体 Xacro](../src/swerve_nav/urdf/robot.urdf.xacro)、[轮组 Xacro](../src/swerve_nav/urdf/wheel.urdf.xacro) 和 [控制器配置](../src/swerve_nav/config/controllers.yaml)。

四个 `joint1` 共用 Xacro 参数 `steering_angle_limit` 定义机械限位与位置命令范围。`WheelConfig::steer_min` 和 `steer_max` 取机械限位与可下发命令范围的交集，按本车要求为 ±2π/3。选解、到位和换向判据统一使用关节位置差。例如从 +119° 转到 −119° 的关节行程是 238°，模 2π 得到的 122° 短弧会越过机械限位。

## 算法与 ROS 2 的职责

- `compute_wheel_velocities()` 保持纯计算函数。第 `i` 轮的转向轴速度为 `(vx − wz*y_i, vy + wz*x_i)`，输出仍是车体系二维速度分量。
- `solve_ik` 负责整组逆解和统一限速。实测舵角、上次命令舵角和已有分支由调用者传入，函数内部没有跨周期状态。
- `DriveControl::step` 负责停稳、预转向、跟踪和状态更新。类保存布局、上一组命令、各轮选解分支和私有阶段。每次调用返回本周期命令和整组逆解是否有解。
- ROS 2 层负责装配配置、按关节名读取 `JointState`、判断反馈与车体命令是否有效，并按控制器轮序下发。驱动反馈用 `speed = omega × radius`，驱动命令用 `omega = speed ÷ radius`。算法内用 rad、m/s、rad/s 和 s。
- 装配时核对有限配置值、正半径、有效舵角区间及本阶段用到的正限值。周期输入需要有限的车体命令、四轮有效反馈和有限且大于零的 `dt`。命令超时且反馈仍有效时走正常减速停车。反馈丢失、坐标越域或时钟跳变时，由 ROS 层请求零驱动、暂停更新舵角目标，恢复后重新接管。零驱动命令只表示已请求停止，停稳仍须由有效反馈确认。

## 函数名与输入输出

`compute_wheel_velocities` 和 `solve_ik` 是可独立调用的计算入口。`solve_ik` 接收选解提示，以便使用实测舵角、上次命令舵角和上次选择的分支。关节角用 rad、滚动线速度用 m/s、周期 `dt` 用 s。

### 当前的数据类型

```cpp
template<class T>
using Wheels = std::array<T, 4>;

enum class Branch { None, Forward, Reverse };

struct WheelState {
    double angle; // 实测舵角，rad
    double speed; // 有符号的实测滚动线速度，m/s
};

struct IkHint {
    double angle;      // 实测舵角，供行程比较
    double hold_angle; // 上次下发的舵角，近零轮速时沿用
    Branch branch;    // 上次整组逆解接受的分支，首次为 None
};

struct IkParams {
    double zero_speed; // 近零轮速阈值，m/s
    double switch_gap; // 切换分支所需节省的机械行程，rad
};

struct WheelGoal {
    WheelTarget target;
    Branch branch;
};

struct StepResult {
    bool reachable;
    Wheels<WheelTarget> cmd;
};
```

`Wheels<T>` 是固定四元素数组的别名，轮序与 `WheelLayout` 一致。`WheelGoal` 记录逆解目标和选解分支，`solve_ik` 用 `std::optional` 表达求解失败。`StepResult::cmd` 始终给出本周期命令，`reachable` 只反映逆解是否有解。有解时也可能正在减速或预转向。无解时 `reachable` 为 `false`，`cmd` 给出减速命令。

### 函数签名与职责

签名中的复合输入均用 `const&` 传递，下表省略引用和常量修饰以便查看数据流。输出按值返回。

| 函数 | 输入类型和参数名 | 返回类型 | 职责 |
| --- | --- | --- | --- |
| `compute_wheel_velocities` | `WheelLayout layout, BodyVelocity velocity` | `Wheels<WheelVelocity>` | 计算各转向轴速度。 |
| `fit_angle` | `double angle, double lo, double hi, double actual` | `std::optional<double>` | 取限位内离实测角最近的等价角。无合法等价角时返回空。 |
| `pick_target` | `WheelVelocity vel, WheelConfig cfg, IkHint hint, IkParams ik` | `std::optional<WheelGoal>` | 选正反转候选。近零轮速时返回保持舵角、轮速为零的有效结果，并沿用分支。 |
| `solve_ik` | `WheelLayout layout, BodyVelocity body, Wheels<IkHint> hints, IkParams ik` | `std::optional<Wheels<WheelGoal>>` | 逐轮选解和统一限速。任一非零轮无解时整组返回空。 |
| `needs_stop` | `Wheels<WheelTarget> prev, Wheels<WheelTarget> goal, Wheels<WheelState> fb, double stop_tol` | `bool` | 检查轮速换向和过大的舵角行程，必要时先停车。 |
| `brake` | `WheelLayout layout, Wheels<WheelTarget> prev, double dt` | `Wheels<WheelTarget>` | 沿用上次下发的舵角，按各轮减速上限求共同系数。 |
| `align_scale` | `Wheels<WheelTarget> goal, Wheels<WheelState> fb` | `double` | 返回由实测舵角误差决定的速度系数上限，范围 `[0, 1]`。 |
| `track` | `WheelLayout layout, Wheels<WheelTarget> prev, Wheels<WheelTarget> goal, double cap, double dt` | `std::optional<Wheels<WheelTarget>>` | 在已通过换向检查的前提下，从 `[0, cap]` 选满足四轮加减速上限的最大正系数。无正系数时返回空。 |
| `DriveControl::step` | `BodyVelocity body, Wheels<WheelState> fb, double dt` | `StepResult` | 读取反馈、组织逆解与周期输出，并保存下一周期使用的命令和状态。待实现。 |

`DriveControl` 构造时接收布局、`IkParams`、停稳容差 `stop_tol`（m/s）和到位容差 `angle_tol`（rad）。`step` 用 `fb` 的实测舵角和 `prev_` 的命令舵角构造 `hints`。首次调用时 `prev_` 为空，从有效反馈建立基准。整组逆解成功后，提取 `WheelGoal::target` 作为 `goal`，同时更新分支记忆。六个辅助函数集中在 `swerve_kinematics.cpp` 的匿名命名空间里。

## 有限命令区间内的选解

1. 用 `std::hypot(vx, vy)` 得到轮速幅值 `s`。当 `s <= ik.zero_speed` 时，直接输出零轮速，舵角沿用 `hint.hold_angle`，分支沿用 `hint.branch`。四轮都满足这个条件时，进入停车流程。轮速近零阈值作用于逆解输入，停稳容差作用于实测轮速。
2. 对非零向量，令 `theta = std::atan2(vy, vx)`，生成 `(theta, +s)` 与 `(theta + π, −s)` 两组候选。两者都满足 `(vx, vy) = speed × (cos(angle), sin(angle))`。各候选在 `angle + 2πk` 中取命令区间内离实测角最近的值。可用 `ceil((min − angle)/2π)` 和 `floor((max − angle)/2π)` 得到整数范围，再把最接近实测角的整数限制在该范围内。边界舍入余量仅用于吸收浮点误差。
3. 在有界位置坐标下，用 `abs(候选角 − 实测角)` 比较实际行程。已有分支可行时，另一分支节省的行程超过 `ik.switch_gap` 才切换。分支为 `None` 时选行程较短的候选，等距时选正转。已有分支无解时选另一合法分支，何时下发由周期流程决定。首版只保留正反转分支，预转向期间若仍出现选解摆动，再验证是否需要引入参考库的运动方向记忆。
4. 对所有非零目标轮速求 `scale = min(1, min_i(drive_speed_i / abs(speed_i)))`，四轮有符号轮速一起乘以 `scale`。一个常数缩放整组目标即可保持速度比例。在理想舵角和无滑动条件下，这等效于按同一比例缩放车体 `(vx, vy, wz)`。近零轮的截断属于额外近似。

±120° 的总行程为 240°。允许滚动轮正反转时，每个非零方向至少有一个合法候选。例如轮速向量方向为 +150° 时，正转候选超限，可以用 −30° 舵角和负轮速实现。验证 `reachable == false` 时使用较窄的测试限位，例如 ±30° 下要求纯横移。跟踪系数区间为空时，目标几何上可达，当前周期应先减速。

## 周期控制

启动或重新接管时，从有效实测舵角和实测轮速建立首周期基准，先等待停稳。基准必须在生成器支持的命令范围内。若舵角坐标不匹配或实际轮速已经超出设计限值，应先由执行层按异常停车流程处理。正常运行后，滚动加减速以**上一周期输出的命令**为基准，选解、舵角到位和停车确认使用**实测反馈**。建议用等待停稳、预转向、跟踪三个内部阶段编排下列行为。

| 内部阶段 | 下发行为与转换条件 |
| --- | --- |
| `WaitingForStop` | 启动、零速指令、整组逆解无解或需停稳的换向都进入此阶段。保持上一组舵角，按共同系数减速。四轮命令归零、实测均停稳且有可执行的非零目标时进入 `Aligning`。 |
| `Aligning` | 确认停稳后才更新舵角目标，全部驱动命令维持零。若反馈重新超出停稳容差，保留已下发的舵角目标，等再次停稳再更新。非零目标轮的实测舵角进入到位容差后才进入跟踪。 |
| `Tracking` | 输出满足本周期约束的整组舵角和轮速。若需停稳换向、对齐系数为零或全组驱动命令已归零，转回 `WaitingForStop`。共同系数暂时无解时可用 `brake()` 降速，仍有滚动且满足跟踪条件时留在 `Tracking`，下一拍用最新目标重算。 |

**换向判定。** 比较上一轮速命令与新目标的非零符号。在某轮上一周期命令刚好为零时，若该轮仍有超过停稳容差的反向实测轮速，也先停车再转向，避免用一拍零命令绕过实测停稳条件。位置行程检查针对两次舵角命令，使用经验证的关节位置表示。

**减速。** 设上一周期有符号轮速为 `p_i`，逐轮减速度幅值为 `d_i`。取共同缩放系数 `b = max(0, max_{p_i≠0}(1 − d_i × dt / abs(p_i)))`，输出 `b × p_i`。如果上一组全零，直接返回全零驱动。每轮轮速都保持原符号或归零，轮组间相对比例保持一致。命令归零后的实测停车仍由反馈确认。

**跟踪。** 设统一限速后的有符号轮速目标为 `q_i`，各轮加速度幅值为 `a_i`。在换向检查之后，求共同系数 `k`，各轮命令为 `k × q_i`。

- 仅对 `q_i ≠ 0` 的轮，取舵角目标和实测舵角的绝对位置差 `e_i`。`align_scale` 以 1 为初始值，取各参与驱动轮的 `cos(min(e_i, π/2))` 与当前值的较小者，`e_i ≥ π/2` 时该轮贡献精确的零。返回值作为 `track` 的 `cap`。这个量按舵角误差收缩滚动速度，地面滑移和车体轨迹误差仍需仿真测量。
- 对 `q_i ≠ 0`，将 `[(abs(p_i) − d_i × dt)/abs(q_i), (abs(p_i) + a_i × dt)/abs(q_i)]` 与全局区间 `[0, k_align]` 取交集。对 `q_i = 0`，直接检查 `abs(p_i) ≤ d_i × dt`。
- 交集存在且最大值大于零时取该最大值。交集为空或仅有零时，保持上一组舵角并按减速规则输出。若命令已降为全零，等实测停稳并重新对齐后再起步。

首版直接下发合法的最终舵角目标。`steer_speed`、`steer_accel` 已有配置字段，若要求软件约束舵角命令的速度与加速度，应在位置目标下发前增加舵角轨迹生成，到位仍按实测舵角与最终目标比较。滚动加减速、停稳容差、到位容差和滞回值需按本车验证。

## 实现顺序与验收

| 步骤 | 可检查的行为 |
| --- | --- |
| 1. 验证逆解 | 覆盖倒车、边界舍入、等距和滞回、±30° 下横移无解以及单轮近零保角。按 `speed × (cos(angle), sin(angle))` 重建各轮向量。 |
| 2. 核对车体几何与统一限速 | 对 `(vx, vy, wz) = (0, 0, 1)`，左前轮向量为 `(−0.31, +0.35)` m/s，右后为 `(+0.31, −0.35)` m/s。对 `(0.31, −0.35, 1)`，左前轮为零，其余三轮仍解算。直行目标为 1.60 m/s 时，按当前模型应统一缩放为 1.0 m/s，对应 10 rad/s。 |
| 3. 验证连续周期 | 反号后先减速，命令归零而反馈未停稳时继续等，舵角未到位时保持零驱动。测试逐轮限值不同、目标轮速归零，以及共同系数区间为空。例如两轮均要到 0.80 m/s，上期分别为 0.60 与 0.10 m/s，当两轮加减速度幅值均为 1 m/s²、`dt = 0.10 s` 时，公共下界 0.625 大于上界 0.25，应沿上一组命令比例减速。 |
| 4. Gazebo 验收 | 先核对展开后的四个 `joint1` 均为 `revolute`，且机械限位和位置命令区间均为 ±2π/3。保持驱动为零，验证接近两端的舵角命令、超限行为以及 +119° 到 −119° 双向切换时的实际行程。再验证停稳、预转向、起步和小幅连续转向，记录舵角误差、实测轮速及车体运动。ROS 两组关节命令经不同话题发送，起步依赖已确认的实测舵角和轮速门槛。 |

参考 [MultiSteerKinematics.Compute](../ref/VehicleMotionControl/VehicleMotionControl/Chassis/MultiSteer/MultiSteerKinematics.cs#L18) 与 [MultiSteerSetpointGenerator](../ref/VehicleMotionControl/VehicleMotionControl/Chassis/MultiSteer/MultiSteerSetpointGenerator.cs#L78) 的单滚动轮路径。历史参考基准见 [OVERVIEW.md](OVERVIEW.md)（`2290738898e6`，当时有本地修改）。本次核对的可读副本以 Git blob `24e90b98` 和 `bef2fdbc` 标识。参考库的常规行程差滞回为 5°，预转向中特定条件下为 10°。其滚动加减速限值全车统一，本方案按当前 `WheelConfig` 的逐轮限值求共同系数。
