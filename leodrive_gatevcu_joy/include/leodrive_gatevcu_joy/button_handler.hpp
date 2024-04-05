#pragma once

#include "leodrive_gatevcu_joy/button.hpp"

namespace leodrive_gatevcu_joy
{

class ButtonHandler
{
public:
  void add_button(Button & button);
  void update(const sensor_msgs::msg::Joy & msg);
  void tick();

private:
  std::vector<Button> buttons_;
};

}  // namespace leodrive_gatevcu_joy
