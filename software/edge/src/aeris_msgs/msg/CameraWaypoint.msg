# Camera pose waypoint for mission planning and gimbal control

builtin_interfaces/Duration time_from_start  # Time offset from mission start
geometry_msgs/Pose pose                      # Camera position and orientation
float32 field_of_view_deg                    # Horizontal FOV for this shot
float32 focal_distance_m                     # Focus distance (0 for infinity)
