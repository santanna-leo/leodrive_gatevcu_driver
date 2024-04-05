#include "leodrive_gatevcu_joy/axis_handler.hpp"
#include "leodrive_gatevcu_joy/button_handler.hpp"
#include "rclcpp/rclcpp.hpp"

#include "leodrive_gatevcu_msgs/msg/longitudinal.hpp"
#include "leodrive_gatevcu_msgs/msg/vehicle.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace leodrive_gatevcu_joy
{

using VehicleMsg = leodrive_gatevcu_msgs::msg::Vehicle;
using LongitudinalMsg = leodrive_gatevcu_msgs::msg::Longitudinal;

class VcuJoyHandler : public rclcpp::Node
{
public:
  explicit VcuJoyHandler(const rclcpp::NodeOptions & options);

private:
  void joy_callback(const sensor_msgs::msg::Joy & msg);
  void state_machine_callback();
  void register_buttons();
  void register_axes();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  rclcpp::Publisher<VehicleMsg>::SharedPtr vehicle_pub_;
  rclcpp::Publisher<LongitudinalMsg>::SharedPtr longitudinal_pub_;

  ButtonHandler button_handler_;
  AxisHandler axis_handler_;

  VehicleMsg vehicle_msg_{};
  LongitudinalMsg longitudinal_msg_{};
};

}  // namespace leodrive_gatevcu_joy
