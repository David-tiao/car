# CAR-LINK V1 communication health; this is status, not motor control.
std_msgs/Header header
bool connected
bool heartbeat_fresh
uint32 peer_session_id
uint8 peer_protocol_minor
uint8 control_owner
uint64 received_frames
uint64 crc_errors
uint64 length_errors
uint64 version_errors
uint64 discarded_bytes
uint64 payload_errors
uint64 received_heartbeats
uint32 peer_reported_crc_errors
uint32 peer_reported_dropped_frames
uint32 peer_reported_tx_errors
