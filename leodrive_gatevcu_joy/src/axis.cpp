#include "leodrive_gatevcu_joy/axis.hpp"

namespace leodrive_gatevcu_joy
{

Axis::Axis(gamepad_axis gamepad_axis) : gamepad_axis_{gamepad_axis}
{
}

void Axis::update_input(const sensor_msgs::msg::Joy & joy_msg)
{
  joy_input_ = joy_msg.axes[gamepad_axis_];
  on_update_.operator()(joy_input_);
  // logger_.log(gamepad_axis_);
}

void Axis::tick()
{
  if (on_tick_.has_value()) {
    on_tick_->operator()();
    // logger_.log(gamepad_axis_);
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

void Axis::set_log_fields(std::string_view field_name, double * field)
{
  logger_.set_log_fields(field_name, field);
}

}  // namespace leodrive_gatevcu_joy