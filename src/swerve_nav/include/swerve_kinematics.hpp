#pragma once
#include <array>
#include <optional>

namespace swerve_nav {

struct WheelConfig {
    double x;      // 转向轴的纵向位置，m，向前为正
    double y;      // 转向轴的横向位置，m，向左为正
    double radius; // 滚动轮半径，m

    double steer_min;   // 最小舵角，rad
    double steer_max;   // 最大舵角，rad
    double steer_speed; // 转向角速度上限，rad/s
    double steer_accel; // 转向加减速共用的角加速度幅值，rad/s²

    double drive_speed; // 滚动线速度幅值上限，m/s
    double drive_accel; // 滚动加速度，m/s²
    double drive_decel; // 滚动减速度，m/s²
};

template <typename T> using Wheels = std::array<T, 4>;
using WheelLayout = Wheels<WheelConfig>;

struct BodyVelocity {
    double vx{}; // 车体前向速度，m/s，向前为正
    double vy{}; // 车体侧向速度，m/s，向左为正
    double wz{}; // 绕车体 z 轴的角速度，rad/s，从上往下看逆时针为正
};

struct WheelVelocity {
    double vx; // 转向轴处的前向速度，m/s
    double vy; // 转向轴处的侧向速度，m/s
};

struct WheelTarget {
    double steer_angle; // 目标舵角，rad，绕转向轴逆时针为正
    double drive_speed; // 目标滚动线速度，m/s，正负表示滚动方向
};

struct WheelState {
    double angle; // 实测舵角，rad
    double speed; // 实测滚动线速度，m/s，正负表示滚动方向
};

enum class Branch {
    None,    // 首次解算，暂无历史分支
    Forward, // 正轮速候选
    Reverse, // 负轮速候选
};

struct IkHint {
    double angle;      // 实测舵角，rad，用来比较转向行程
    double hold_angle; // 上次下发的舵角，rad，轮速接近零时沿用
    Branch branch;     // 上次选中的分支，首次传 Branch::None
};

struct IkParams {
    double zero_speed; // 近零轮速阈值，m/s
    double switch_gap; // 切换分支至少要节省的转向行程，rad
};

struct WheelGoal {
    WheelTarget target; // 本次选出的目标舵角和带符号轮速
    Branch branch;      // 本次选解采用的分支
};

Wheels<WheelVelocity> compute_wheel_velocities(
    const WheelLayout &layout, const BodyVelocity &velocity);

std::optional<Wheels<WheelGoal>> solve_ik(const WheelLayout &layout, const BodyVelocity &body,
    const Wheels<IkHint> &hints, const IkParams &ik);

struct StepResult {
    bool reachable;          // 本周期的整组逆解是否有解
    Wheels<WheelTarget> cmd; // 本周期实际下发的四轮命令
};

class DriveControl {
  public:
    DriveControl(const WheelLayout &layout, const IkParams &ik, double stop_tol, double angle_tol);

    StepResult step(const BodyVelocity &body, const Wheels<WheelState> &fb, double dt);

  private:
    enum class Phase {
        WaitingForStop,
        Aligning,
        Tracking,
    };

    WheelLayout layout_;
    IkParams ik_;
    double stop_tol_;  // 实测轮速的停稳容差，m/s
    double angle_tol_; // 实测舵角的到位容差，rad

    std::optional<Wheels<WheelTarget>> prev_; // 上一周期下发的命令
    Wheels<Branch> branches_{};               // 各轮上次选中的分支，初始为 None
    Phase phase_ = Phase::WaitingForStop;
};

} // namespace swerve_nav
