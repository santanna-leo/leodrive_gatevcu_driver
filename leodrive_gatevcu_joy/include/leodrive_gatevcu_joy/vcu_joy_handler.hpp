#include "leodrive_gatevcu_joy/button_handler.hpp"
#include "rclcpp/rclcpp.hpp"

#include "leodrive_gatevcu_msgs/msg/vehicle.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace leodrive_gatevcu_joy
{

using VehicleMsg = leodrive_gatevcu_msgs::msg::Vehicle;

class VcuJoyHandler : public rclcpp::Node
{
public:
  explicit VcuJoyHandler(const rclcpp::NodeOptions & options);

private:
  void joy_callback(const sensor_msgs::msg::Joy & msg);
  void state_machine_callback();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  rclcpp::Publisher<VehicleMsg>::SharedPtr vehicle_pub_;

  ButtonHandler button_handler_;
};

}  // namespace leodrive_gatevcu_joy
