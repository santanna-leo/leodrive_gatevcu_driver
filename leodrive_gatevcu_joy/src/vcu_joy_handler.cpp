#include "leodrive_gatevcu_joy/vcu_joy_handler.hpp"

namespace leodrive_gatevcu_joy
{
VcuJoyHandler::VcuJoyHandler(const rclcpp::NodeOptions & options) : Node{"vcu_sender", options}
{
  RCLCPP_INFO_STREAM(get_logger(), "Hello");
}

void VcuJoyHandler::joy_callback(const sensor_msgs::msg::Joy & msg)
{
}

}  // namespace leodrive_gatevcu_joy

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(leodrive_gatevcu_joy::VcuJoyHandler)