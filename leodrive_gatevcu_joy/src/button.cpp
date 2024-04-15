#include "leodrive_gatevcu_joy/button.hpp"

namespace leodrive_gatevcu_joy
{

Button::Button(gamepad_button gamepad_button) : gamepad_button_{gamepad_button}
{
}

Button::Button(gamepad_axes_button gamepad_axes_button) : gamepad_axes_button_{gamepad_axes_button}
{
}

void Button::update_input(const sensor_msgs::msg::Joy & joy_msg)
{
  const bool is_pressed = check_pressed(joy_msg);

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
  logger_.set_log_fields(field_name, field);
}

bool Button::check_pressed(const sensor_msgs::msg::Joy & msg)
{
  if (gamepad_button_.has_value()) {
    return msg.buttons[*gamepad_button_] > 0;
  } else {
    switch (*gamepad_axes_button_) {
      case gamepad_axes_button::UP_BUTTON:
        return msg.axes[7] > 0;
      case gamepad_axes_button::DOWN_BUTTON:
        return msg.axes[7] < 0;
    }
  }
  return false;
}

void Button::log_status()
{
  if (gamepad_button_.has_value())
    logger_.log(*gamepad_button_);
  else
    logger_.log(*gamepad_axes_button_);
}

}  // namespace leodrive_gatevcu_joy