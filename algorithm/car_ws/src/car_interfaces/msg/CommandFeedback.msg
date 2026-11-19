# CAR-LINK V1 B2: command acceptance/lifecycle. ACK is NOT completion.
std_msgs/Header header
uint32 source_time_ms
uint8 event_type               # 0x90-0x96
uint32 cmd_id
uint8 original_command_type
uint8 status_code
uint8 progress_percent
uint8 phase_code
uint16 error_code
uint8 control_owner
uint8 chassis_mode
uint8 run_state
