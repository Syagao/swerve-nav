#include "swerve_kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace swerve_nav {

namespace {

std::optional<double> fit_angle(double angle, double lo, double hi, double actual) {
    constexpr double period = 2.0 * std::numbers::pi;
    constexpr double tolerance = 1e-9;
    const double k_min = std::ceil((lo - angle - tolerance) / period);
    const double k_max = std::floor((hi - angle + tolerance) / period);
    if (k_min > k_max)
        return std::nullopt;
    const double nearest_k = std::round((actual - angle) / period);
    const double k = std::clamp(nearest_k, k_min, k_max);
    const double fitted = angle + k * period;
    return std::clamp(fitted, lo, hi);
}

std::optional<WheelGoal> pick_target(
    const WheelVelocity &vel, const WheelConfig &cfg, const IkHint &hint, const IkParams &ik) {
    const double speed = std::hypot(vel.vx, vel.vy);
    if (speed <= ik.zero_speed) {
        return WheelGoal{{hint.hold_angle, 0.0}, hint.branch};
    }
    const double theta = std::atan2(vel.vy, vel.vx);
    const auto forward = fit_angle(theta, cfg.steer_min, cfg.steer_max, hint.angle);
    const auto reverse =
        fit_angle(theta + std::numbers::pi, cfg.steer_min, cfg.steer_max, hint.angle);
    if (!forward && !reverse) {
        return std::nullopt;
    }
    if (!forward) {
        return WheelGoal{{*reverse, -speed}, Branch::Reverse};
    }
    if (!reverse) {
        return WheelGoal{{*forward, speed}, Branch::Forward};
    }
    const double forward_travel = std::abs(*forward - hint.angle);
    const double reverse_travel = std::abs(*reverse - hint.angle);
    bool use_forward = forward_travel <= reverse_travel;
    if (hint.branch == Branch::Forward) {
        use_forward = forward_travel - reverse_travel <= ik.switch_gap;
    } else if (hint.branch == Branch::Reverse) {
        use_forward = reverse_travel - forward_travel > ik.switch_gap;
    }
    if (use_forward) {
        return WheelGoal{{*forward, speed}, Branch::Forward};
    }
    return WheelGoal{{*reverse, -speed}, Branch::Reverse};
}

bool needs_stop(const Wheels<WheelTarget> &prev, const Wheels<WheelTarget> &goal,
    const Wheels<WheelState> &fb, double stop_tol) {
    for (std::size_t i = 0; i < prev.size(); ++i) {
        const double prev_speed = prev[i].drive_speed;
        const double goal_speed = goal[i].drive_speed;
        const double actual_speed = fb[i].speed;
        const bool command_reverses =
            (prev_speed > 0.0 && goal_speed < 0.0) || (prev_speed < 0.0 && goal_speed > 0.0);
        const bool feedback_reverses =
            prev_speed == 0.0 && ((actual_speed > stop_tol && goal_speed < 0.0) ||
                                     (actual_speed < -stop_tol && goal_speed > 0.0));
        const double steer_travel = std::abs(goal[i].steer_angle - prev[i].steer_angle);
        if (command_reverses || feedback_reverses || steer_travel > std::numbers::pi) {
            return true;
        }
    }
    return false;
}

Wheels<WheelTarget> brake(const WheelLayout &layout, const Wheels<WheelTarget> &prev, double dt) {
    double scale = 0.0;
    for (std::size_t i = 0; i < prev.size(); ++i) {
        const double speed = std::abs(prev[i].drive_speed);
        if (speed == 0.0)
            continue;
        const double max_drop = layout[i].drive_decel * dt;
        const double min_scale = 1.0 - max_drop / speed;
        scale = std::max(scale, min_scale);
    }
    auto next = prev;
    for (auto &target : next) {
        target.drive_speed *= scale;
    }
    return next;
}

double align_scale(const Wheels<WheelTarget> &goal, const Wheels<WheelState> &fb) {
    double scale = 1.0;
    for (std::size_t i = 0; i < goal.size(); ++i) {
        if (goal[i].drive_speed == 0.0)
            continue;
        const double error = std::abs(goal[i].steer_angle - fb[i].angle);
        if (error >= std::numbers::pi / 2.0)
            return 0.0;
        scale = std::min(scale, std::cos(error));
    }
    return scale;
}

std::optional<Wheels<WheelTarget>> track(const WheelLayout &layout, const Wheels<WheelTarget> &prev,
    const Wheels<WheelTarget> &goal, double cap, double dt) {
    double lower = 0.0;
    double upper = cap;
    for (std::size_t i = 0; i < goal.size(); ++i) {
        const double prev_speed = std::abs(prev[i].drive_speed);
        const double goal_speed = std::abs(goal[i].drive_speed);
        const double max_drop = layout[i].drive_decel * dt;
        const double max_rise = layout[i].drive_accel * dt;
        if (goal_speed == 0.0) {
            if (prev_speed > max_drop) {
                return std::nullopt;
            }
            continue;
        }
        lower = std::max(lower, (prev_speed - max_drop) / goal_speed);
        upper = std::min(upper, (prev_speed + max_rise) / goal_speed);
    }
    if (lower > upper || upper <= 0.0) {
        return std::nullopt;
    }
    auto next = goal;
    for (auto &target : next) {
        target.drive_speed *= upper;
    }
    return next;
}

} // namespace

Wheels<WheelVelocity> compute_wheel_velocities(
    const WheelLayout &layout, const BodyVelocity &velocity) {
    std::array<WheelVelocity, 4> result;
    for (std::size_t i = 0; i < layout.size(); ++i) {
        const auto &wheel = layout[i];
        result[i] = {
            .vx = velocity.vx - velocity.wz * wheel.y,
            .vy = velocity.vy + velocity.wz * wheel.x,
        };
    }
    return result;
}

std::optional<Wheels<WheelGoal>> solve_ik(const WheelLayout &layout, const BodyVelocity &body,
    const Wheels<IkHint> &hints, const IkParams &ik) {
    const auto velocities = compute_wheel_velocities(layout, body);
    Wheels<WheelGoal> goals;
    double scale = 1.0;
    for (std::size_t i = 0; i < layout.size(); ++i) {
        const auto &wheel = layout[i];
        const auto goal = pick_target(velocities[i], wheel, hints[i], ik);
        if (!goal) {
            return std::nullopt;
        }
        goals[i] = *goal;
        const double speed = std::abs(goal->target.drive_speed);
        if (speed > wheel.drive_speed) {
            scale = std::min(scale, wheel.drive_speed / speed);
        }
    }
    for (auto &goal : goals) {
        goal.target.drive_speed *= scale;
    }
    return goals;
}

} // namespace swerve_nav
