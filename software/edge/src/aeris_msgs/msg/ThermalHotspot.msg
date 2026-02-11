# Detected thermal anomaly from infrared imaging for fire monitoring and search operations.
# Published by perception pipeline when hotspots exceed temperature thresholds.

builtin_interfaces/Time stamp  # Detection time for temporal correlation with other sensors
uint32[4] bbox_px              # Bounding box [x, y, width, height] in source image pixels
float32 temp_c                 # Estimated temperature in Celsius for severity assessment
float32 confidence             # Detection confidence [0.0, 1.0] from the neural network
string frame_id                # Coordinate frame of the source thermal camera image
