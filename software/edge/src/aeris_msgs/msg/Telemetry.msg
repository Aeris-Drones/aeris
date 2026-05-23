# Comprehensive vehicle telemetry for real-time tracking and flight safety monitoring.
# Aggregates position, attitude, and velocity into a single message for efficient broadcasting.

string vehicle_id                  # Unique vehicle identifier for fleet management
string vehicle_type                # Platform classification: "multirotor", "fixedwing", "vtol"
builtin_interfaces/Time timestamp  # Message generation time for latency analysis
aeris_msgs/GeoPosition position    # WGS84 position for geospatial tracking
aeris_msgs/VehicleOrientation orientation  # Roll/pitch/yaw for attitude awareness
geometry_msgs/Vector3 velocity     # NED frame velocity in m/s for trajectory prediction
float32 battery_percent            # Remaining battery percentage in the range [0, 100]
float32 remaining_flight_time_sec  # Remaining battery endurance in seconds when available
bool remaining_flight_time_available  # True when remaining_flight_time_sec is trustworthy telemetry
