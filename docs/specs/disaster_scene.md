# Disaster Scene Notes

`software/sim/worlds/disaster_scene.sdf` provides the baseline search-and-rescue environment used for SITL validation.

## Included environment elements

- Collapsed building meshes and debris clusters
- Rescue zone and hazard beacon props
- Cinematic camera rig include (`model://cinematic_camera`)

## Story 2.1 mapping focus

- Scout stereo + IMU topics are bridged into ROS 2 for OpenVINS + RTAB-Map.
- Validation targets the frame chain `map -> odom -> base_link`.
- Loop-closure evidence is captured via `/rtabmap/info` artifacts.
