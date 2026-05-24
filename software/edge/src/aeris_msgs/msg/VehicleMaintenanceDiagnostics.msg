# Canonical maintenance-only diagnostic snapshot for one vehicle.

uint8 STATE_UNKNOWN=0
uint8 STATE_HEALTHY=1
uint8 STATE_WARNING=2
uint8 STATE_BLOCKED=3

string vehicle_id
uint8 motor_health_state
string motor_health_detail
uint8 calibration_state
string calibration_detail
uint8 imu_state
string imu_detail
uint8 compass_state
string compass_detail
uint8 accelerometer_state
string accelerometer_detail
