#pragma once

#include "leodrive_gatevcu_joy/axis_types.hpp"
#include "leodrive_gatevcu_joy/logger.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

namespace leodrive_gatevcu_joy
{

inline double mapOneRangeToAnother(
  double sourceNumber, double fromA, double fromB, double toA, double toB, int decimalPrecision)
{
  if (std::abs(sourceNumber - fromA) < 1e-9) {
    return toA;
  } else if (std::abs(sourceNumber - fromB) < 1e-9) {
    return toB;
  }
  double deltaA = fromB - fromA;
  double deltaB = toB - toA;
  double scale = deltaB / deltaA;
  double negA = -1 * fromA;
  double offset = (negA * scale) + toA;
  double finalNumber = (sourceNumber * scale) + offset;
  int calcScale = static_cast<int>(std::pow(10, decimalPrecision));
  return static_cast<double>(std::round(finalNumber * calcScale) / calcScale);
}

class Axis
{
public:
  explicit Axis(gamepad_axis gamepad_axis);
  void update_input(const sensor_msgs::msg::Joy & joy_msg);
  void tick();
  void on_update(const std::function<void(const float & joy_input)> & function);
  void on_tick(const std::function<void()> & function);
  void set_log_fields(std::string_view field_name, double * field);

private:
  Logger<double> logger_;
  gamepad_axis gamepad_axis_;

  std::function<void(const float & joy_input)> on_update_;
  std::optional<std::function<void()>> on_tick_;

  float joy_input_{};
  std::string field_name_;
};

}  // namespace leodrive_gatevcu_joy
