#include "leodrive_gatevcu_joy/axis_handler.hpp"

namespace leodrive_gatevcu_joy
{

void AxisHandler::add_axis(Axis & axis)
{
  axes_.push_back(axis);
}

void AxisHandler::update(const sensor_msgs::msg::Joy & msg)
{
  std::for_each(axes_.begin(), axes_.end(), [msg](Axis & axis) { axis.update_input(msg); });
}

void AxisHandler::tick()
{
  std::for_each(axes_.begin(), axes_.end(), [](Axis & axis) { axis.tick(); });
}

}  // namespace leodrive_gatevcu_joy