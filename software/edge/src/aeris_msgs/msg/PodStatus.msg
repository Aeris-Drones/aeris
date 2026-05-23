# Canonical pod lifecycle state published by the device manager.

uint8 STATE_DETECTED=0
uint8 STATE_VALIDATING=1
uint8 STATE_POWER_CHECK=2
uint8 STATE_SOFT_START=3
uint8 STATE_ENUMERATING=4
uint8 STATE_REGISTERED=5
uint8 STATE_REJECTED=6
uint8 STATE_DISCONNECTED=7
uint8 STATE_FAULTED=8

string vehicle_id
string slot_id
string one_wire_id
string pod_serial
string pod_type
uint8 lifecycle_state
string lifecycle_state_label
string[] capabilities
bool connected
bool power_ready
bool link_ready
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
float32 enumeration_elapsed_sec
aeris_msgs/PodStageTiming[] stage_timings
string rejection_code
string rejection_detail
string fault_code
string fault_detail
