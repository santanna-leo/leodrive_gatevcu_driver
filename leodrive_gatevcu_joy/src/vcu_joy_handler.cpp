#include "leodrive_gatevcu_joy/vcu_joy_handler.hpp"

namespace leodrive_gatevcu_joy
{
VcuJoyHandler::VcuJoyHandler(const rclcpp::NodeOptions & options) : Node{"vcu_sender", options}
{
  RCLCPP_INFO_STREAM(get_logger(), "Hello");

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&VcuJoyHandler::joy_callback, this, std::placeholders::_1));
  state_machine_timer_ = create_wall_timer(
    rclcpp::Rate(100).period(), std::bind(&VcuJoyHandler::state_machine_callback, this));

  vehicle_pub_ = create_publisher<VehicleMsg>("vehicle", rclcpp::SensorDataQoS());
  VehicleMsg vehicle_msg{};

  Button myButton{gamepad::CIRCLE_BUTTON};
  myButton.on_click([this]() { RCLCPP_INFO_STREAM(this->get_logger(), "My click."); });
  myButton.on_hold([this]() { RCLCPP_INFO_STREAM(this->get_logger(), "My hold."); });

  button_handler_.add_button(myButton);
}

void VcuJoyHandler::joy_callback(const sensor_msgs::msg::Joy & msg)
{
  button_handler_.update(msg);
}

void VcuJoyHandler::state_machine_callback()
{
  button_handler_.tick();
}

}  // namespace leodrive_gatevcu_joy

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(leodrive_gatevcu_joy::VcuJoyHandler)