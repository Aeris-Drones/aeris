# SLAM Simulation Parity Parameters

This document captures the simulation-only wiring values that remain outside core SLAM node logic.

## Topic Contract (ROS 2)

- `/scout1/stereo/left/image_raw`
- `/scout1/stereo/right/image_raw`
- `/scout1/stereo/left/camera_info`
- `/scout1/stereo/right/camera_info`
- `/scout1/imu/data`
- `/scout1/openvins/odom`
- `/map`
- `/rtabmap/cloud_map`

## Frame Tree Contract

- `map -> odom -> base_link`
- Left camera frame: `stereo_left_optical_frame`
- Right camera frame: `stereo_right_optical_frame`
- IMU frame: `imu_link`

## Synchronization and Noise Assumptions

- Stereo and IMU should be timestamp-aligned with max slop <= `10ms`
- IMU publish rate target: `>= 200Hz`
- Camera publish rate target: `>= 15Hz` per eye
- OpenVINS + RTAB-Map should run with `use_sim_time=true`

## Configuration Sources

- OpenVINS params: `software/edge/src/aeris_map/config/openvins_sim.yaml`
- RTAB-Map params: `software/edge/src/aeris_map/config/rtabmap_vio_sim.yaml`
- Combined launch: `software/edge/src/aeris_map/launch/rtabmap_vio_sim.launch.py`

## Guardrail

No simulation-specific branches are allowed in core processing nodes. Any environment differences must stay in launch/config/scripts.
