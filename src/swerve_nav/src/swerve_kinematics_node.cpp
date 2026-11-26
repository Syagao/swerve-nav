#include "swerve_kinematics.hpp"

#include <algorithm>
#include <angles/angles.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <stdexcept>
#include <string>
#include <urdf/model.hpp>

namespace swerve_nav {
namespace {
constexpr std::array prefixes{"front_left", "front_right", "rear_left", "rear_right"};

WheelLayout load_layout(const std::string &robot_description) {
    urdf::Model model;
    if (!model.initString(robot_description)) {
        throw std::runtime_error("无法解析 robot_description");
    }

    const auto base_to_body = model.getJoint("base_to_body_joint");
    if (!base_to_body || base_to_body->type != urdf::Joint::FIXED ||
        base_to_body->parent_link_name != "base_link" ||
        base_to_body->child_link_name != "body_link") {
        throw std::runtime_error("缺少 base_link 到 body_link 的固定关节");
    }
    const auto &body_pose = base_to_body->parent_to_joint_origin_transform;
    if (body_pose.rotation.x != 0.0 || body_pose.rotation.y != 0.0 || body_pose.rotation.z != 0.0) {
        throw std::runtime_error("当前布局解析要求 body_link 与 base_link 同向");
    }

    WheelLayout layout;

    for (std::size_t i = 0; i < layout.size(); ++i) {
        const std::string prefix = prefixes[i];
        const auto steer = model.getJoint(prefix + "_joint1");
        const auto drive = model.getJoint(prefix + "_joint2");

        if (!steer || !drive || !steer->limits || !drive->limits ||
            steer->parent_link_name != "body_link" ||
            drive->parent_link_name != steer->child_link_name) {
            throw std::runtime_error(prefix + " 的关节结构或限值有误");
        }

        const auto wheel = model.getLink(drive->child_link_name);
        if (!wheel || !wheel->collision || !wheel->collision->geometry ||
            wheel->collision->geometry->type != urdf::Geometry::CYLINDER) {
            throw std::runtime_error(prefix + " 缺少圆柱形轮胎碰撞体");
        }

        const auto &tire = static_cast<const urdf::Cylinder &>(*wheel->collision->geometry);
        const double radius = tire.radius;
        const auto &s = *steer->limits;
        const auto &d = *drive->limits;

        for (const double value : std::array{radius, s.velocity, s.acceleration, s.deceleration,
                 d.velocity, d.acceleration, d.deceleration}) {
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::runtime_error(prefix + " 的半径或速度、加减速限值无效");
            }
        }
        if (!std::isfinite(s.lower) || !std::isfinite(s.upper) || s.lower >= s.upper) {
            throw std::runtime_error(prefix + " 的舵角限位无效");
        }

        const auto &position = steer->parent_to_joint_origin_transform.position;
        layout[i] = {
            .x = body_pose.position.x + position.x,
            .y = body_pose.position.y + position.y,
            .radius = radius,
            .steer_min = s.lower,
            .steer_max = s.upper,
            .steer_speed = s.velocity,
            .steer_accel = std::min(s.acceleration, s.deceleration),
            .drive_speed = d.velocity * radius,
            .drive_accel = d.acceleration * radius,
            .drive_decel = d.deceleration * radius,
        };
    }

    return layout;
}

class SwerveKinematicsNode : public rclcpp::Node {
  public:
    SwerveKinematicsNode(const IkParams &ik, double stop_tol, double angle_tol)
        : rclcpp::Node("swerve_kinematics"),
          layout_(load_layout(declare_parameter<std::string>("robot_description"))),
          drive_control_(layout_, ik, stop_tol, angle_tol) {
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState &msg) { on_joint_state(msg); });
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1),
            [this](const geometry_msgs::msg::Twist &msg) { on_cmd_vel(msg); });
        steering_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/steering_controller/commands", rclcpp::QoS(1));
        drive_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/drive_controller/commands", rclcpp::QoS(1));
        control_timer_ =
            create_timer(std::chrono::milliseconds(20), [this]() { on_control_tick(); });
    }

  private:
    WheelLayout layout_;
    DriveControl drive_control_;
    std::optional<Wheels<WheelState>> wheel_feedback_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::optional<BodyVelocity> body_command_;
    std::optional<rclcpp::Time> last_control_time_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

  private:
    void on_joint_state(const sensor_msgs::msg::JointState &msg);
    void on_cmd_vel(const geometry_msgs::msg::Twist &msg);
    void on_control_tick();
};

void SwerveKinematicsNode::on_joint_state(const sensor_msgs::msg::JointState &msg) {
    if (msg.position.size() != msg.name.size() || msg.velocity.size() != msg.name.size()) {
        return;
    }

    Wheels<WheelState> feedback{};

    for (std::size_t i = 0; i < feedback.size(); ++i) {
        const std::string prefix = prefixes[i];
        const auto steer = std::find(msg.name.begin(), msg.name.end(), prefix + "_joint1");
        const auto drive = std::find(msg.name.begin(), msg.name.end(), prefix + "_joint2");

        if (steer == msg.name.end() || drive == msg.name.end()) {
            return;
        }

        const auto steer_index = steer - msg.name.begin();
        const auto drive_index = drive - msg.name.begin();
        const double angle = msg.position[steer_index];
        const double speed = msg.velocity[drive_index] * layout_[i].radius;

        if (!std::isfinite(angle) || !std::isfinite(speed)) {
            return;
        }

        feedback[i] = {
            .angle = angle,
            .speed = speed,
        };
    }
    wheel_feedback_ = feedback;
}

void SwerveKinematicsNode::on_cmd_vel(const geometry_msgs::msg::Twist &msg) {
    const BodyVelocity command{
        .vx = msg.linear.x,
        .vy = msg.linear.y,
        .wz = msg.angular.z,
    };

    if (!std::isfinite(command.vx) || !std::isfinite(command.vy) || !std::isfinite(command.wz)) {
        return;
    }

    body_command_ = command;
}

void SwerveKinematicsNode::on_control_tick() {
    if (!body_command_ || !wheel_feedback_) {
        return;
    }
    const auto current_time = now();
    if (!last_control_time_) {
        last_control_time_ = current_time;
        return;
    }
    const double dt = (current_time - *last_control_time_).seconds();
    last_control_time_ = current_time;
    if (dt <= 0.0) {
        return;
    }
    const StepResult result = drive_control_.step(*body_command_, *wheel_feedback_, dt);
    std_msgs::msg::Float64MultiArray steering_msg;
    std_msgs::msg::Float64MultiArray drive_msg;
    steering_msg.data.resize(result.cmd.size());
    drive_msg.data.resize(result.cmd.size());
    for (std::size_t i = 0; i < result.cmd.size(); ++i) {
        steering_msg.data[i] = result.cmd[i].steer_angle;
        drive_msg.data[i] = result.cmd[i].drive_speed / layout_[i].radius;
    }
    steering_pub_->publish(steering_msg);
    drive_pub_->publish(drive_msg);
}

} // namespace
} // namespace swerve_nav

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    const swerve_nav::IkParams ik{
        .zero_speed = 0.005,
        .switch_gap = angles::from_degrees(5.0),
    };
    constexpr double stop_tol = 0.02; // m/s
    const double angle_tol = angles::from_degrees(1.0);

    auto node = std::make_shared<swerve_nav::SwerveKinematicsNode>(ik, stop_tol, angle_tol);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
