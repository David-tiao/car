# Mirrors algorithm/openmv/main.py ASCII lines such as:
# #SEQ,156123516231; #OBS,22,17,STACK,STORE,5,2,158,122
# #OBJ,color,u,v,w,h,area,rotation; #STATE,step,state,picks,places.
# This is a DATA CONTRACT only. Current STM32F4 vision.c expects a different V4
# framed protocol. No production publisher until the protocols are unified.
std_msgs/Header header
string event_type                 # SEQ/SLOT/OBJ/OBS/STATE/LOST/ERR/RESULT_OK/...
string raw_line
string task_code                 # 12 contiguous digits, when event_type == SEQ
uint8 step
uint32 token
string operation                 # PICK/PLACE/STACK/HOME
string station                   # RAW/ROUGH/STORE/HOME
uint8 color_id                   # 1 red,2 yellow,3 blue,4 green,5 black,6 light-blue
uint8 ring_id                    # 0 unnumbered; 1..3 numbered ring
float64 pixel_u
float64 pixel_v
float64 radius_px
bool pixel_position_valid
bool metric_position_valid       # currently false without camera/body calibration
string state_name
