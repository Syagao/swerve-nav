#include "swerve_kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace swerve_nav {

std::array<WheelVelocity, 4> compute_wheel_velocities(
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
    const WheelVelocity &vel, const WheelConfig &cfg, const IkHint &hint, const IkParams &ik) {}

std::array<WheelTarget, 4> inverse_kinematics(
    const WheelLayout &layout, const BodyVelocity &velocity) {}

} // namespace swerve_nav
