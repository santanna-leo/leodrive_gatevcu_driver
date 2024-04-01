#include "leodrive_gatevcu_driver/vcu_sender.hpp"

namespace leodrive_gatevcu_driver
{
VcuSender::VcuSender(const rclcpp::NodeOptions & options) : Node{"vcu_sender", options}
{
}
}  // namespace leodrive_gatevcu_driver

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(leodrive_gatevcu_driver::VcuSender)