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
- Map tile params: `software/edge/src/aeris_map/config/map_tile_stream.yaml`
- Combined launch: `software/edge/src/aeris_map/launch/rtabmap_vio_sim.launch.py`

## SLAM Mode Control and Verification

- Runtime selector:
  - `slam_mode:=vio` enables the RTAB-Map + OpenVINS adapter (default)
  - `slam_mode:=liosam` is registered but intentionally fails fast as `not implemented`
- Launch example:

```bash
ros2 launch aeris_map rtabmap_vio_sim.launch.py slam_mode:=vio
```

- Verify selected mode in mission progress payload:

```bash
ros2 topic echo /mission/progress --once
```

Expected metadata key:
- `vehicleSlamModes.<vehicle_id>: "vio"` for scout vehicles in this phase

- Validate deterministic fallback guardrail:

```bash
ros2 run aeris_map map_tile_publisher --ros-args -p slam_mode:=liosam
```

Expected behavior:
- node exits with explicit `not-implemented` startup failure
- no silent fallback to `vio`

## Phase 2 Migration Constraints (LIO-SAM)

- Keep map-service external outputs unchanged: `/map`, `/rtabmap/cloud_map`, `/map/tiles`, `/map/get_tile_bytes`.
- Preserve frame contract: `map -> odom -> base_link`.
- Require explicit LiDAR + IMU topic wiring before enabling runtime support; do not auto-fallback to `vio`.

## Guardrail

No simulation-specific branches are allowed in core processing nodes. Any environment differences must stay in launch/config/scripts.
