#include "leodrive_gatevcu_joy/press_detector.hpp"

namespace leodrive_gatevcu_joy
{

void PressDetector::is_pressed(const bool & input)
{
  if (current_state_) {
    if (!input) current_state_ = false;
  } else {
    if (input) current_state_ = true;
  }
}

}  // namespace leodrive_gatevcu_joy