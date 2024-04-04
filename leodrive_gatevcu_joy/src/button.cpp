#include "leodrive_gatevcu_joy/button.hpp"

namespace leodrive_gatevcu_joy
{

Button::Button(rclcpp::Node & node) : node_{node}
{
}

void Button::update_input(const bool & is_pressed)
{
  switch (current_press_state_) {
    case PressState::NOT_PRESSED:
      if (is_pressed) {
        current_press_state_ = PressState::PRESSING;
        current_press_change_state_ = PressChangeState::PRESSED;
        press_time_ = node_.now();
      }
      break;
    case PressState::PRESSING:
      if (!is_pressed) {
        current_press_state_ = PressState::NOT_PRESSED;
        current_press_change_state_ = PressChangeState::RELEASED;
      }
      break;
  }
}

void Button::tick()
{
  if (!press_time_.has_value()) return;

  if (current_press_state_ == PressState::PRESSING) {
    const auto delta_time = node_.now() - *press_time_;
    if (delta_time > hold_duration && current_button_state_ != ButtonState::HOLDING) {
      current_button_state_ = ButtonState::HOLD;
      current_press_change_state_ = PressChangeState::IDLE;
    }
  }

  if (current_press_change_state_ == PressChangeState::RELEASED) {
    if (current_button_state_ != ButtonState::HOLDING) {
      current_button_state_ = ButtonState::CLICK;
    } else {
      current_button_state_ = ButtonState::IDLE;
      current_press_change_state_ = PressChangeState::IDLE;
    }
  }

  switch (current_button_state_) {
    case ButtonState::CLICK:
      on_click_();
      current_button_state_ = ButtonState::IDLE;
      current_press_change_state_ = PressChangeState::IDLE;
      break;
    case ButtonState::HOLD:
      on_hold_();
      current_button_state_ = ButtonState::HOLDING;
      break;
    case ButtonState::IDLE:
    case ButtonState::HOLDING:
      break;
  }
}

void Button::on_click(const std::function<void()> & function)
{
  on_click_ = function;
}

void Button::on_hold(const std::function<void()> & function)
{
  on_hold_ = function;
}

}  // namespace leodrive_gatevcu_joy