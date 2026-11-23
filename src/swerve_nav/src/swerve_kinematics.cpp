#include "swerve_kinematics.hpp"

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

std::array<WheelTarget, 4> inverse_kinematics(
    const WheelLayout &layout, const BodyVelocity &velocity) {}

} // namespace swerve_nav
