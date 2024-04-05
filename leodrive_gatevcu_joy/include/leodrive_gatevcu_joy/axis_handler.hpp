#pragma once

#include "leodrive_gatevcu_joy/axis.hpp"

namespace leodrive_gatevcu_joy
{

class AxisHandler
{
public:
  void add_axis(Axis & axis);
  void update(const sensor_msgs::msg::Joy & msg);
  void tick();

private:
  std::vector<Axis> axes_;
};

}  // namespace leodrive_gatevcu_joy
