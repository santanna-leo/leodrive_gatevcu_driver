#include "leodrive_gatevcu_adapter/vcu_sender.hpp"

namespace leodrive_gatevcu_adapter
{
VcuSender::VcuSender(const rclcpp::NodeOptions & options) : Node{"vcu_sender", options}
{
  RCLCPP_INFO_STREAM(get_logger(), "Hello");

  can_frame_pub_ = create_publisher<CanFrame>("/to_can_bus", 500);

  steering_sub_ = create_subscription<SteeringMsg>(
    "steering_wheel", rclcpp::SensorDataQoS(),
    std::bind(&VcuSender::steering_callback, this, std::placeholders::_1));

  longitudinal_sub_ = create_subscription<LongitudinalMsg>(
    "longitudinal", rclcpp::SensorDataQoS(),
    std::bind(&VcuSender::longitudinal_callback, this, std::placeholders::_1));
}

void VcuSender::steering_callback(const SteeringMsg & msg)
{
  auto can_frame = create_frame();

  FrontWheelCommands_t cmds{};
  cmds.set_steering_wheel_angle_ro = msg.angle;
  cmds.set_steering_wheel_torque = msg.torque;

  can_frame.id = Pack_FrontWheelCommands_drivedb(
    &cmds, can_frame.data.data(), &can_frame.dlc,
    reinterpret_cast<uint8_t *>(&can_frame.is_extended));

  can_frame_pub_->publish(can_frame);
}

void VcuSender::longitudinal_callback(const LongitudinalMsg & msg)
{
  auto can_frame = create_frame();

  LongitudinalCommandsV2_t cmds{};
  cmds.set_gas_pedal_pos_ro = msg.gas_pedal;
  cmds.set_brake_force_ro = msg.brake_pedal;

  can_frame.id = Pack_LongitudinalCommandsV2_drivedb(
    &cmds, can_frame.data.data(), &can_frame.dlc,
    reinterpret_cast<uint8_t *>(&can_frame.is_extended));

  can_frame_pub_->publish(can_frame);
}

CanFrame VcuSender::create_frame()
{
  CanFrame can_frame{};
  can_frame.header.frame_id = frame_id;
  can_frame.header.stamp = now();
  can_frame.is_rtr = false;
  can_frame.is_error = false;
  return can_frame;
}

}  // namespace leodrive_gatevcu_adapter

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(leodrive_gatevcu_adapter::VcuSender)