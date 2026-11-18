# ch2 is race state only if STM32 was compiled APP_MODE_RACE.
# In other build modes ch2 contains Square_GetState().
std_msgs/Header header
uint32 source_time_ms
uint8 phase_code                  # ch1, 0 idle,1 move,2 turn,3 settle/scan,4 finished
uint8 status_code                 # ch2
uint8 action_index                # ch3
uint8 error_code                  # ch6
uint8 plan_index                 # ch7
bool plan_done                   # ch9
bool race_state_semantics_known  # false by default; external build flag needed
