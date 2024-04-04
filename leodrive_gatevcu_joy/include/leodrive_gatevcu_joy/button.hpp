#pragma once

#include "rclcpp/rclcpp.hpp"

namespace leodrive_gatevcu_joy
{

constexpr std::chrono::milliseconds hold_duration{200};

class Button
{
public:
  explicit Button(rclcpp::Node & node);
  void update_input(const bool & is_pressed);
  void tick();
  void on_click(const std::function<void()> & function);
  void on_hold(const std::function<void()> & function);

private:
  rclcpp::Node & node_;
  std::optional<rclcpp::Time> press_time_;

  enum class PressState { NOT_PRESSED, PRESSING };
  enum class PressChangeState { IDLE, PRESSED, RELEASED };
  enum class ButtonState { IDLE, CLICK, HOLD, HOLDING };
  PressState current_press_state_{PressState::NOT_PRESSED};
  PressChangeState current_press_change_state_{PressChangeState::IDLE};
  ButtonState current_button_state_{ButtonState::IDLE};

  std::function<void()> on_click_;
  std::function<void()> on_hold_;
};

}  // namespace leodrive_gatevcu_joy
