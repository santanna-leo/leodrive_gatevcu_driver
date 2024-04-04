#pragma once

namespace leodrive_gatevcu_joy
{

class PressDetector
{
public:
  void is_pressed(const bool & input);

private:
  bool last_state_{false};
  bool current_state_{false};
};

}  // namespace leodrive_gatevcu_joy
