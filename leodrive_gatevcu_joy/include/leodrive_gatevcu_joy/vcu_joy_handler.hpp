#include "rclcpp/rclcpp.hpp"

namespace leodrive_gatevcu_joy
{

class VcuJoyHandler : public rclcpp::Node
{
public:
  explicit VcuJoyHandler(const rclcpp::NodeOptions & options);
};

}  // namespace leodrive_gatevcu_joy
