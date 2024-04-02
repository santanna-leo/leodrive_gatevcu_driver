# Blinker states
uint8 BLINKER_OFF = 0
uint8 BLINKER_LEFT = 1
uint8 BLINKER_RIGHT = 2
uint8 BLINKER_HAZARD = 3

# Wiper states
uint8 WIPER_OFF = 0
uint8 WIPER_ON = 1

# Gear states
uint8 GEAR_P = 1
uint8 GEAR_R = 2
uint8 GEAR_N = 3
uint8 GEAR_D = 4

# Mode states
uint8 MODE_OFF = 0
uint8 MODE_ON = 1

# Handbrake states
uint8 HANDBRAKE_IGNORE = 0
uint8 HANDBRAKE_RELEASE = 1
uint8 HANDBRAKE_PULL = 2

std_msgs/Header header
uint8 blinker
uint8 head_light
uint8 wiper
uint8 gear
uint8 mode
uint8 hand_brake
uint8 takeover_request
uint8 long_mode
