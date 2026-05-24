# Canonical firmware update snapshot for one vehicle.

uint8 STATE_UNKNOWN=0
uint8 STATE_IDLE=1
uint8 STATE_DOWNLOADING=2
uint8 STATE_VALIDATING=3
uint8 STATE_APPLYING=4
uint8 STATE_VERIFYING=5
uint8 STATE_COMPLETE=6
uint8 STATE_FAILED=7
uint8 STATE_ROLLING_BACK=8
uint8 STATE_ROLLED_BACK=9

string vehicle_id
string package_id
string current_version
string target_version
uint8 lifecycle_state
string lifecycle_state_label
float32 progress_percent
string active_slot
string inactive_slot
bool rollback_performed
string status_detail
string error_code
string error_detail
