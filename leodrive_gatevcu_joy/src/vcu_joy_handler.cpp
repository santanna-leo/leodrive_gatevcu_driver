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

  register_buttons();
}

void VcuJoyHandler::joy_callback(const sensor_msgs::msg::Joy & msg)
{
  button_handler_.update(msg);
}

void VcuJoyHandler::state_machine_callback()
{
  button_handler_.tick();
}

void VcuJoyHandler::register_buttons()
{
  Button wiper{gamepad::X_BUTTON};
  wiper.set_log_fields("wiper", &vehicle_msg_.wiper);
  wiper.on_click([this]() {
    if (vehicle_msg_.wiper == VehicleMsg::WIPER_OFF)
      vehicle_msg_.wiper = VehicleMsg::WIPER_ON;
    else
      vehicle_msg_.wiper = VehicleMsg::WIPER_OFF;
  });
  button_handler_.add_button(wiper);

  Button mode{gamepad::PS4_BUTTON};
  mode.set_log_fields("mode", &vehicle_msg_.mode);
  mode.on_click([this]() {
    if (vehicle_msg_.mode == VehicleMsg::MODE_OFF)
      vehicle_msg_.mode = VehicleMsg::MODE_ON;
    else
      vehicle_msg_.mode = VehicleMsg::MODE_OFF;
  });
  button_handler_.add_button(mode);

  Button handbrake{gamepad::SQUARE_BUTTON};
  handbrake.set_log_fields("handbrake", &vehicle_msg_.hand_brake);
  handbrake.on_click([this]() { vehicle_msg_.hand_brake = VehicleMsg::HANDBRAKE_PULL; });
  handbrake.on_hold([this]() { vehicle_msg_.hand_brake = VehicleMsg::HANDBRAKE_RELEASE; });
  button_handler_.add_button(handbrake);
}

}  // namespace leodrive_gatevcu_joy

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(leodrive_gatevcu_joy::VcuJoyHandler)