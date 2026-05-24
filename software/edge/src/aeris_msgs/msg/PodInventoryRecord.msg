# Canonical fleet pod inventory row for the maintenance viewer.

uint8 CALIBRATION_UNKNOWN=0
uint8 CALIBRATION_CURRENT=1
uint8 CALIBRATION_DUE_SOON=2
uint8 CALIBRATION_OVERDUE=3

string pod_serial
string pod_type
string one_wire_id
bool attached
string vehicle_id
string slot_id
string lifecycle_state_label
bool connected
bool power_ready
bool link_ready
string[] capabilities
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
builtin_interfaces/Time last_calibration
builtin_interfaces/Time next_calibration_due
uint8 calibration_state
string calibration_state_label
string calibration_detail
