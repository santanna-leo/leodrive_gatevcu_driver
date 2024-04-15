#pragma once

#include "leodrive_gatevcu_joy/axis_types.hpp"
#include "leodrive_gatevcu_joy/button_types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace leodrive_gatevcu_joy
{

template <typename field_t>
class Logger
{
public:
  explicit Logger() : logger_{rclcpp::get_logger("gatevcu_joy")} {}

  void set_log_fields(std::string_view field_name, field_t * field)
  {
    field_name_ = field_name;
    field_ = field;
  }

  void log(const gamepad_button & button_type)
  {
    const auto field = std::to_string(*field_);
    RCLCPP_INFO(
      logger_, "%s (%s): %s", button_to_string(button_type).c_str(), field_name_.c_str(),
      field.c_str());
  }

  void log(const gamepad_axes_button & button_type)
  {
    const auto field = std::to_string(*field_);
    RCLCPP_INFO(
      logger_, "%s (%s): %s", button_to_string(button_type).c_str(), field_name_.c_str(),
      field.c_str());
  }

  void log(const gamepad_axis & axis_type)
  {
    const auto field = std::to_string(*field_);
    RCLCPP_INFO(
      logger_, "%s (%s): %s", axis_to_string(axis_type).c_str(), field_name_.c_str(),
      field.c_str());
  }

private:
  rclcpp::Logger logger_;
  std::string field_name_;
  field_t * field_{};
};

}  // namespace leodrive_gatevcu_joy
