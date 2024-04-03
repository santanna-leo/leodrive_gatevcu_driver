#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

namespace leodrive_gatevcu_joy
{

class VcuJoyHandler : public rclcpp::Node
{
public:
  explicit VcuJoyHandler(const rclcpp::NodeOptions & options);

private:
  void joy_callback(const sensor_msgs::msg::Joy & msg);
  void state_machine_callback();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;
};

}  // namespace leodrive_gatevcu_joy
