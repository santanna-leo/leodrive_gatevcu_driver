#include "leodrive_gatevcu_joy/vcu_joy_handler.hpp"

namespace leodrive_gatevcu_joy
{
VcuJoyHandler::VcuJoyHandler(const rclcpp::NodeOptions & options)
: Node{"vcu_sender", options}, btn_(*this)
{
  RCLCPP_INFO_STREAM(get_logger(), "Hello");

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&VcuJoyHandler::joy_callback, this, std::placeholders::_1));
  state_machine_timer_ = create_wall_timer(
    rclcpp::Rate(100).period(), std::bind(&VcuJoyHandler::state_machine_callback, this));

  btn_.on_click([this]() { RCLCPP_INFO_STREAM(this->get_logger(), "Clicked."); });
  btn_.on_hold([this]() { RCLCPP_INFO_STREAM(this->get_logger(), "Holded."); });
}

void VcuJoyHandler::joy_callback(const sensor_msgs::msg::Joy & msg)
{
  btn_.update_input(msg.buttons[0] > 0);
}

void VcuJoyHandler::state_machine_callback()
{
  btn_.tick();
}

}  // namespace leodrive_gatevcu_joy

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(leodrive_gatevcu_joy::VcuJoyHandler)