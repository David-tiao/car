# A derived, read-only state from /car/vision/event.
std_msgs/Header header
bool event_stream_fresh
bool observation_fresh
bool target_visible
bool qr_valid
string task_code
string last_event_type
string operation
string station
uint8 step
uint32 token
float64 pixel_u
float64 pixel_v
uint64 events_received
