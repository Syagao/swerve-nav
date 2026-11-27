#include <SDL.h>
#include <algorithm>
#include <angles/angles.h>
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>

namespace {

std::optional<geometry_msgs::msg::Twist> command_for_key(
    char key, double linear_speed, double angular_speed) {
    geometry_msgs::msg::Twist command{};

    switch (key) {
    case 'w':
        command.linear.x = linear_speed;
        break;
    case 's':
        command.linear.x = -linear_speed;
        break;
    case 'a':
        command.linear.y = linear_speed;
        break;
    case 'd':
        command.linear.y = -linear_speed;
        break;
    case 'q':
        command.angular.z = angular_speed;
        break;
    case 'e':
        command.angular.z = -angular_speed;
        break;
    case ' ':
        break;
    default:
        return std::nullopt;
    }

    return command;
}

class KeyboardTeleop : public rclcpp::Node {
  public:
    KeyboardTeleop()
        : rclcpp::Node("keyboard_teleop"),
          cmd_vel_pub_(create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1))),
          linear_speed_(declare_parameter<double>("linear_speed_mps")),
          angular_speed_(
              angles::from_degrees(declare_parameter<double>("angular_speed_deg_per_sec"))),
          angular_accel_limit_(
              angles::from_degrees(declare_parameter<double>("angular_accel_deg_per_sec2"))),
          command_period_(declare_parameter<std::int64_t>("publish_period_ms")) {
        command_timer_ = create_wall_timer(command_period_, [this]() { on_command_tick(); });
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::set<char> pressed_keys_;
    bool stop_pending_ = false;
    rclcpp::TimerBase::SharedPtr command_timer_;
    double published_angular_speed_ = 0.0; // rad/s
    const double linear_speed_;
    const double angular_speed_;
    const double angular_accel_limit_;
    const std::chrono::milliseconds command_period_;

  private:
    void on_key_event(char key, bool pressed);
    void publish_active_command();
    bool poll_keyboard_events();
    void on_command_tick();
};

void KeyboardTeleop::on_key_event(char key, bool pressed) {
    if (!pressed) {
        if (pressed_keys_.erase(key) != 0 && pressed_keys_.empty()) {
            stop_pending_ = true;
        }
        return;
    }
    if (key == ' ') {
        pressed_keys_.clear();
        published_angular_speed_ = 0.0;
        stop_pending_ = true;
        return;
    }
    if (command_for_key(key, linear_speed_, angular_speed_)) {
        pressed_keys_.insert(key);
    }
}

void KeyboardTeleop::publish_active_command() {
    if (pressed_keys_.empty() && !stop_pending_ && published_angular_speed_ == 0.0) {
        return;
    }
    geometry_msgs::msg::Twist command{};
    for (char key : pressed_keys_) {
        const auto contribution = command_for_key(key, linear_speed_, angular_speed_);
        command.linear.x += contribution->linear.x;
        command.linear.y += contribution->linear.y;
        command.angular.z += contribution->angular.z;
    }
    const double period_sec = std::chrono::duration<double>(command_period_).count();
    const double max_change = angular_accel_limit_ * period_sec;
    published_angular_speed_ = std::clamp(command.angular.z, published_angular_speed_ - max_change,
        published_angular_speed_ + max_change);
    command.angular.z = published_angular_speed_;
    cmd_vel_pub_->publish(command);
    stop_pending_ = false;
}

bool KeyboardTeleop::poll_keyboard_events() {
    SDL_Event event{};
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            on_key_event(' ', true);
            return false;
        }
        if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_FOCUS_LOST) {
            on_key_event(' ', true);
            continue;
        }
        if (event.type != SDL_KEYDOWN && event.type != SDL_KEYUP) {
            continue;
        }
        if (event.key.repeat != 0) {
            continue;
        }
        const auto key = event.key.keysym.sym;
        if ((key >= SDLK_a && key <= SDLK_z) || key == SDLK_SPACE) {
            on_key_event(static_cast<char>(key), event.type == SDL_KEYDOWN);
        }
    }
    return true;
}

void KeyboardTeleop::on_command_tick() {
    const bool keep_running = poll_keyboard_events();
    publish_active_command();
    if (!keep_running) {
        rclcpp::shutdown();
    }
}

} // namespace

int main(int argc, char *argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL 初始化失败：" << SDL_GetError() << '\n';
        return 1;
    }
    SDL_Window *window = SDL_CreateWindow("Swerve Keyboard Teleop", SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED, 480, 180, SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "创建键盘窗口失败：" << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleop>();
    rclcpp::spin(node);
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
