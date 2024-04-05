#pragma once

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

namespace leodrive_gatevcu_joy
{

using clock = std::chrono::system_clock;
constexpr std::chrono::milliseconds hold_duration{200};
enum gamepad {
  X_BUTTON,
  CIRCLE_BUTTON,
  TRIANGLE_BUTTON,
  SQUARE_BUTTON,
  LEFT_BUTTON,
  RIGHT_BUTTON,
  LEFT_TRIGGER,
  RIGHT_TRIGGER,
  SHARE_BUTTON,
  OPTIONS_BUTTON,
  PS4_BUTTON,
  LEFT_JOYSTICK_BUTTON,
  RIGHT_JOYSTICK_BUTTON
};

class Button
{
public:
  explicit Button(gamepad gamepadButton);
  void update_input(const sensor_msgs::msg::Joy & joy_msg);
  void tick();
  void on_click(const std::function<void()> & function);
  void on_hold(const std::function<void()> & function);
  void set_log_fields(std::string_view field_name, uint8_t * field);

private:
  void log_status();
  static std::string button_to_string(gamepad button);

  rclcpp::Logger logger_;

  gamepad gamepad_button_;
  std::optional<std::chrono::time_point<clock>> press_time_;

  enum class PressState { NOT_PRESSED, PRESSING };
  enum class PressChangeState { IDLE, PRESSED, RELEASED };
  enum class ButtonState { IDLE, CLICK, HOLD, HOLDING };
  PressState current_press_state_{PressState::NOT_PRESSED};
  PressChangeState current_press_change_state_{PressChangeState::IDLE};
  ButtonState current_button_state_{ButtonState::IDLE};

  std::optional<std::function<void()>> on_click_;
  std::optional<std::function<void()>> on_hold_;

  std::string field_name_;
  uint8_t * field_{};
};

}  // namespace leodrive_gatevcu_joy
