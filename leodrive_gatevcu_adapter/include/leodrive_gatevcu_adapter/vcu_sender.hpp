#include "leodrive_gatevcu_adapter/can_interface/drivedb.h"
#include "rclcpp/rclcpp.hpp"

#include "can_msgs/msg/frame.hpp"
#include "leodrive_gatevcu_msgs/msg/steering_wheel.hpp"

namespace leodrive_gatevcu_adapter
{

using CanFrame = can_msgs::msg::Frame;
using SteeringMsg = leodrive_gatevcu_msgs::msg::SteeringWheel;

constexpr std::string_view frame_id{"can"};

class VcuSender : public rclcpp::Node
{
public:
  explicit VcuSender(const rclcpp::NodeOptions & options);
  void steering_callback(const SteeringMsg & msg);

private:
  rclcpp::Publisher<CanFrame>::SharedPtr can_frame_pub_;
  rclcpp::Subscription<SteeringMsg>::SharedPtr steering_sub_;
};

}  // namespace leodrive_gatevcu_adapter
