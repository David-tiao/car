# Four wheel order exactly: FL, FR, RL, RR.
# Pulse/s from current STM32F4 Motor_t, NOT motor angular rate.
std_msgs/Header header
uint32 source_time_ms
float64[4] target_pulse_per_s     # ch27, ch35, ch36, ch37
float64[4] actual_pulse_per_s     # ch28, ch29, ch30, ch31
float64[4] target_mps             # converted using pulses_per_meter
float64[4] actual_mps             # converted using pulses_per_meter
float64 max_abs_pid_output        # ch38 aggregate maximum only
bool per_wheel_pid_output_valid   # current CSV does not log four independent outputs
bool encoder_counts_valid         # current CSV does not log accumulated encoder counts
