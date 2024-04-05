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
  Button wiper{gamepad_button::X_BUTTON};
  wiper.set_log_fields("wiper", &vehicle_msg_.wiper);
  wiper.on_click([this]() {
    if (vehicle_msg_.wiper == VehicleMsg::WIPER_OFF)
      vehicle_msg_.wiper = VehicleMsg::WIPER_ON;
    else
      vehicle_msg_.wiper = VehicleMsg::WIPER_OFF;
  });
  button_handler_.add_button(wiper);

  Button mode{gamepad_button::PS4_BUTTON};
  mode.set_log_fields("mode", &vehicle_msg_.mode);
  mode.on_click([this]() {
    if (vehicle_msg_.mode == VehicleMsg::MODE_OFF)
      vehicle_msg_.mode = VehicleMsg::MODE_ON;
    else
      vehicle_msg_.mode = VehicleMsg::MODE_OFF;
  });
  button_handler_.add_button(mode);

  Button handbrake{gamepad_button::SQUARE_BUTTON};
  handbrake.set_log_fields("handbrake", &vehicle_msg_.hand_brake);
  handbrake.on_click([this]() { vehicle_msg_.hand_brake = VehicleMsg::HANDBRAKE_PULL; });
  handbrake.on_hold([this]() { vehicle_msg_.hand_brake = VehicleMsg::HANDBRAKE_RELEASE; });
  button_handler_.add_button(handbrake);

  Button left_blinker{gamepad_button::LEFT_BUTTON};
  left_blinker.set_log_fields("left blinker", &vehicle_msg_.blinker);
  left_blinker.on_click([this]() {
    if (vehicle_msg_.blinker == VehicleMsg::BLINKER_LEFT)
      vehicle_msg_.blinker = VehicleMsg ::BLINKER_OFF;
    else {
      vehicle_msg_.blinker = VehicleMsg ::BLINKER_LEFT;
    }
  });
  button_handler_.add_button(left_blinker);

  Button right_blinker{gamepad_button::RIGHT_BUTTON};
  right_blinker.set_log_fields("right blinker", &vehicle_msg_.blinker);
  right_blinker.on_click([this]() {
    if (vehicle_msg_.blinker == VehicleMsg::BLINKER_RIGHT)
      vehicle_msg_.blinker = VehicleMsg ::BLINKER_OFF;
    else {
      vehicle_msg_.blinker = VehicleMsg ::BLINKER_RIGHT;
    }
  });
  button_handler_.add_button(right_blinker);

  Button hazard_blinker{gamepad_button::TRIANGLE_BUTTON};
  hazard_blinker.set_log_fields("hazard blinker", &vehicle_msg_.blinker);
  hazard_blinker.on_click([this]() {
    if (vehicle_msg_.blinker == VehicleMsg::BLINKER_HAZARD)
      vehicle_msg_.blinker = VehicleMsg ::BLINKER_OFF;
    else {
      vehicle_msg_.blinker = VehicleMsg ::BLINKER_HAZARD;
    }
  });
  button_handler_.add_button(hazard_blinker);

  Button gear_up{gamepad_axes_button::UP_BUTTON};
  gear_up.set_log_fields("gear up", &vehicle_msg_.gear);
  gear_up.on_click([this]() {
    if (vehicle_msg_.gear < 4) {
      ++vehicle_msg_.gear;
    }
  });
  button_handler_.add_button(gear_up);

  Button gear_down{gamepad_axes_button::DOWN_BUTTON};
  gear_down.set_log_fields("gear down", &vehicle_msg_.gear);
  gear_down.on_click([this]() {
    if (vehicle_msg_.gear > 1) {
      --vehicle_msg_.gear;
    }
  });
  button_handler_.add_button(gear_down);
}

}  // namespace leodrive_gatevcu_joy

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(leodrive_gatevcu_joy::VcuJoyHandler)