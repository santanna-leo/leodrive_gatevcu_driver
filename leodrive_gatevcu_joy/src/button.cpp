#include "leodrive_gatevcu_joy/button.hpp"

namespace leodrive_gatevcu_joy
{

Button::Button(gamepad_button gamepad_button)
: logger_{rclcpp::get_logger("button")}, gamepad_button_{gamepad_button}
{
}

void Button::update_input(const sensor_msgs::msg::Joy & joy_msg)
{
  const bool is_pressed = joy_msg.buttons[gamepad_button_] > 0;

  switch (current_press_state_) {
    case PressState::NOT_PRESSED:
      if (is_pressed) {
        current_press_state_ = PressState::PRESSING;
        current_press_change_state_ = PressChangeState::PRESSED;
        press_time_ = clock::now();
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
    const auto delta_time = clock::now() - *press_time_;
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
      if (on_click_.has_value()) {
        on_click_->operator()();
        log_status();
      }
      current_button_state_ = ButtonState::IDLE;
      current_press_change_state_ = PressChangeState::IDLE;
      break;
    case ButtonState::HOLD:
      if (on_hold_.has_value()) {
        on_hold_->operator()();
        log_status();
      }
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

void Button::set_log_fields(std::string_view field_name, uint8_t * field)
{
  field_name_ = field_name;
  field_ = field;
}

void Button::log_status()
{
  RCLCPP_INFO(
    logger_, "%s (%s): %d", button_to_string(gamepad_button_).c_str(), field_name_.c_str(),
    *field_);
}

std::string Button::button_to_string(gamepad_button button)
{
  switch (button) {
    case X_BUTTON:
      return "X Button";
    case CIRCLE_BUTTON:
      return "Circle Button";
    case TRIANGLE_BUTTON:
      return "Triangle Button";
    case SQUARE_BUTTON:
      return "Square Button";
    case LEFT_BUTTON:
      return "Left Button";
    case RIGHT_BUTTON:
      return "Right Button";
    case LEFT_TRIGGER_BUTTON:
      return "Left Trigger Button";
    case RIGHT_TRIGGER_BUTTON:
      return "Right Trigger Button";
    case SHARE_BUTTON:
      return "Share Button";
    case OPTIONS_BUTTON:
      return "Options Button";
    case PS4_BUTTON:
      return "PS4 Button";
    case LEFT_JOYSTICK_BUTTON:
      return "Left Joystick Button";
    case RIGHT_JOYSTICK_BUTTON:
      return "Right Joystick Button";
    default:
      return "Unknown Button";
  }
}

}  // namespace leodrive_gatevcu_joy