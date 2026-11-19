# Reports source quality, not position estimation or a motor command.
std_msgs/Header header
bool source_fresh
bool any_position_valid
bool operation_active
bool fault_active
uint16 fault_code
bool gripper_valid
bool turntable_valid
bool lift_valid
bool extension_valid
uint64 samples_received
