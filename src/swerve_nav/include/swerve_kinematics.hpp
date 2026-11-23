#pragma once
#include <array>

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

using WheelLayout = std::array<WheelConfig, 4>;

struct BodyVelocity {
    double vx{}; // 车体前向速度，m/s，向前为正
    double vy{}; // 车体侧向速度，m/s，向左为正
    double wz{}; // 绕车体 z 轴的角速度，rad/s，从上往下看逆时针为正
};

struct WheelVelocity {
    double vx; // 转向轴处的前向速度，m/s
    double vy; // 转向轴处的侧向速度，m/s
};

std::array<WheelVelocity, 4> compute_wheel_velocities(
    const WheelLayout &layout, const BodyVelocity &velocity);

struct WheelTarget {
    double steer_angle; // 目标舵角，rad，绕转向轴逆时针为正
    double drive_speed; // 目标滚动线速度，m/s，正负表示滚动方向
};

std::array<WheelTarget, 4> inverse_kinematics(
    const WheelLayout &layout, const BodyVelocity &velocity);

} // namespace swerve_nav
