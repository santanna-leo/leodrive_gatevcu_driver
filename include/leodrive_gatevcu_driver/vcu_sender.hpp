#include "rclcpp/rclcpp.hpp"

namespace leodrive_gatevcu_driver
{

class VcuSender : public rclcpp::Node
{
public:
  explicit VcuSender(const rclcpp::NodeOptions & options);
};

}  // namespace leodrive_gatevcu_driver
