#include "leodrive_gatevcu_adapter/vcu_sender.hpp"

namespace leodrive_gatevcu_adapter
{
VcuSender::VcuSender(const rclcpp::NodeOptions & options) : Node{"vcu_sender", options}
{
  RCLCPP_INFO_STREAM(get_logger(), "Hello");

  can_frame_pub_ = create_publisher<can_msgs::msg::Frame>("/to_can_bus", rclcpp::SensorDataQoS());

  steering_sub_ = create_subscription<leodrive_gatevcu_msgs::msg::SteeringWheel>(
    "steering_wheel", rclcpp::SensorDataQoS(),
    std::bind(&VcuSender::steering_callback, this, std::placeholders::_1));
}

void VcuSender::steering_callback(const leodrive_gatevcu_msgs::msg::SteeringWheel & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "Angle: " << msg.angle);
  RCLCPP_INFO_STREAM(get_logger(), "Torque: " << msg.torque);
}

}  // namespace leodrive_gatevcu_adapter

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(leodrive_gatevcu_adapter::VcuSender)