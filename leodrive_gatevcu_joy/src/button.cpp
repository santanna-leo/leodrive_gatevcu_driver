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
  switch (current_press_state_) {
    case PressState::PRESSING: {
      // Check hold time
      const auto delta_time = node_.now() - *press_time_;
      if (delta_time > hold_duration) {
        current_button_state_ = ButtonState::HOLD;  // onHold()
        current_press_change_state_ = PressChangeState::IDLE;
        // current_press_state_ = PressState::NOT_PRESSED;
      }
      break;
    }
    case PressState::NOT_PRESSED:
      // RCLCPP_INFO_STREAM(node_.get_logger(), "        Not pressed");
      break;
  }

  switch (current_press_change_state_) {
    case PressChangeState::PRESSED:
      // RCLCPP_INFO_STREAM(node_.get_logger(), "        Pressed");
      break;
    case PressChangeState::RELEASED:
      RCLCPP_INFO_STREAM(node_.get_logger(), "        Released");
      if (current_button_state_ != ButtonState::HOLD) {
        RCLCPP_INFO_STREAM(
          node_.get_logger(),
          "current_button_state_: "
            << static_cast<std::underlying_type<ButtonState>::type>(current_button_state_));
        current_button_state_ = ButtonState::CLICK;
        break;
      }
      current_press_change_state_ = PressChangeState::IDLE;
      current_button_state_ = ButtonState::IDLE;

      break;
    case PressChangeState::IDLE:
      // RCLCPP_INFO_STREAM(node_.get_logger(), "        IDLEEEE");
      break;
  }

  switch (current_button_state_) {
    case ButtonState::CLICK:
      RCLCPP_INFO_STREAM(node_.get_logger(), "  ----- Click");
      current_button_state_ = ButtonState::IDLE;
      current_press_change_state_ = PressChangeState::IDLE;
      current_press_state_ = PressState::NOT_PRESSED;
      break;
    case ButtonState::HOLD:
      RCLCPP_INFO_STREAM(node_.get_logger(), "  ***** Hold");
      break;
    case ButtonState::IDLE:

      break;
  }
}

}  // namespace leodrive_gatevcu_joy