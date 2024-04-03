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

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

}  // namespace leodrive_gatevcu_joy
