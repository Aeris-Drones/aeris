# Vehicle attitude in NED frame following aerospace convention.
# Used for gimbal pointing, camera georeferencing, and flight control feedback.

float64 roll   # Rotation about X axis (forward), positive right wing down, radians
float64 pitch  # Rotation about Y axis (right), positive nose up, radians
float64 yaw    # Rotation about Z axis (down), positive clockwise from north, radians
