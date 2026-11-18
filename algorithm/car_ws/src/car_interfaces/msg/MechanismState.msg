# STM32F4 actuators exist, but their positions are NOT recorded in ULog43.
# Each validity bit MUST be false until separately acquired from firmware.
std_msgs/Header header
uint32 source_time_ms
bool gripper_valid
float64 gripper_angle_deg
bool turntable_valid
float64 turntable_angle_deg
bool lift_valid
float64 lift_height_mm
bool extension_valid
float64 extension_mm
bool operation_active
uint16 fault_code
