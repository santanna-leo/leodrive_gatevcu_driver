#include "rclcpp/rclcpp.hpp"

#include "can_msgs/msg/frame.hpp"
#include "leodrive_gatevcu_msgs/msg/steering_wheel.hpp"

namespace leodrive_gatevcu_adapter
{

class VcuSender : public rclcpp::Node
{
public:
  explicit VcuSender(const rclcpp::NodeOptions & options);
  void steering_callback(const leodrive_gatevcu_msgs::msg::SteeringWheel & msg);

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;
  rclcpp::Subscription<leodrive_gatevcu_msgs::msg::SteeringWheel>::SharedPtr steering_sub_;
};

}  // namespace leodrive_gatevcu_adapter
