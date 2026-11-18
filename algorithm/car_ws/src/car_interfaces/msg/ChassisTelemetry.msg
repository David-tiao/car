# Ground truth source: STM32F4/Control/mecanum_chassis.h and Hardware/bsp_ulog.c.
# Header timestamp = Pi receive time; source_time_ms = MCU HAL_GetTick, not ROS epoch.
std_msgs/Header header
uint32 source_time_ms
uint8 chassis_mode                 # ch0: 0 speed, 1 position
bool target_reached               # ch4
float64 x_m                       # ch32: odometry position
float64 y_m                       # ch33
float64 yaw_rad                   # ch16 deg -> rad
float64 vx_mps                    # reconstructed from ch28..ch31
float64 vy_mps                    # reconstructed from ch28..ch31
float64 wz_radps                  # ch15, body yaw rate
float64 wheel_estimated_wz_radps  # reconstructed from ch28..ch31
float64 target_x_m                # ch21
float64 target_y_m                # ch22
float64 target_yaw_rad            # ch23 deg -> rad
float64 target_wz_radps           # ch14
float64 target_speed_mps          # magnitude ONLY, ch19 * 0.073632
bool target_linear_components_valid # false for current ULog43: ch12/ch13 formatted %.0f
float64 x_target_error_m         # ch24: target X minus current X
float64 y_target_error_m         # ch25: target Y minus current Y
float64 yaw_target_error_rad     # ch42 deg -> rad
float64 lateral_target_error_m    # ch39: goal lateral error, NOT path cross-track error
float64 driver_voltage_v         # ch40
float64 board_voltage_v          # ch41
float64 travelled_distance_m      # ch34
float64 imu_yaw_rad               # ch17, independent validity not recorded
bool imu_valid                    # false when reading ULog43, unknown from source
