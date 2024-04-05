#include "leodrive_gatevcu_joy/axis.hpp"

namespace leodrive_gatevcu_joy
{

Axis::Axis(gamepad_axis gamepad_axis) : gamepad_axis_{gamepad_axis}
{
}

void Axis::update_input(const sensor_msgs::msg::Joy & joy_msg)
{
  joy_input_ = joy_msg.axes[gamepad_axis_];
  on_update_(joy_input_);
}

void Axis::tick()
{
  if (on_tick_.has_value()) {
    on_tick_->operator()();
  }
}

void Axis::on_update(const std::function<void(const float & joy_input)> & function)
{
  on_update_ = function;
}

void Axis::on_tick(const std::function<void()> & function)
{
  on_tick_ = function;
}

std::string Axis::axis_to_string(gamepad_axis axis)
{
  switch (axis) {
    case LEFT_JOYSTICK_HORIZONTAL:
      return "Left Joystick Horizontal";
    case LEFT_JOYSTICK_VERTICAL:
      return "Left Joystick Vertical";
    case LEFT_TRIGGER:
      return "Left Trigger";
    case RIGHT_JOYSTICK_HORIZONTAL:
      return "Right Trigger Horizontal";
    case RIGHT_JOYSTICK_VERTICAL:
      return "Right Joystick Vertical";
    case RIGHT_TRIGGER:
      return "Right Trigger";
    case DPAD_HORIZONTAL:
      return "DPad Horizontal";
    case DPAD_VERTICAL:
      return "DPad Vertical";
    default:
      return "Unknown Axis";
  }
}

}  // namespace leodrive_gatevcu_joy