#include "leodrive_gatevcu_joy/button_handler.hpp"

namespace leodrive_gatevcu_joy
{

void ButtonHandler::add_button(Button & button)
{
  buttons_.push_back(button);
}

void ButtonHandler::update(const sensor_msgs::msg::Joy & msg)
{
  std::for_each(
    buttons_.begin(), buttons_.end(), [msg](Button & button) { button.update_input(msg); });
}

void ButtonHandler::tick()
{
  std::for_each(buttons_.begin(), buttons_.end(), [](Button & button) { button.tick(); });
}

}  // namespace leodrive_gatevcu_joy