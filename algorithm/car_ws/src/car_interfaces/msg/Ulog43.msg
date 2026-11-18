# Native STM32F4 ULog43, 43 columns without header. No invented sensor values.
# valid_semantics marks channels documented and actively filled by the firmware.
# Placeholder zeros are NOT evidence that an unrecorded sensor is healthy.
std_msgs/Header header
uint32 source_time_ms
float64[43] channels
bool[43] valid_semantics
string original_line
