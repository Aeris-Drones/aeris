# RTAB-Map + OpenVINS Simulation Runbook

## Launch Sequence

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch software/sim/launch/multi_drone_sim.launch.py world:=software/sim/worlds/disaster_scene.sdf scout_model_name:=scout1
ros2 launch aeris_map rtabmap_vio_sim.launch.py scout_model_name:=scout1 slam_mode:=vio
```

For parallel scouts or repeated runs, set unique outputs per scout/run:
- `openvins_log_directory:=/tmp/aeris/openvins/<scout_or_run_id>`
- `rtabmap_database_path:=/tmp/aeris/rtabmap/<scout_or_run_id>.db`

## SLAM Mode Runtime Verification

Confirm mission-progress metadata reports active mode:

```bash
ros2 topic echo /mission/progress --once
```

Expected key:
- `vehicleSlamModes.scout_1: "vio"` (and same mode for additional scouts in this phase)

Validate guardrail for deferred LIO-SAM adapter:

```bash
ros2 run aeris_map map_tile_publisher --ros-args -p slam_mode:=liosam
```

Expected behavior:
- startup fails with a `not implemented` message
- process does not silently switch back to `vio`

## Required Smoke Checks

```bash
SCOUT_NAME=scout1 ./software/sim/tools/validate_slam_topics.sh
```

This checks:
- stereo image and camera-info topics are active
- IMU and OpenVINS odometry are active
- map and cloud outputs are active
- camera/IMU/odom publish rates meet minimum thresholds (defaults: camera >= 15 Hz, IMU >= 200 Hz, odom >= 15 Hz)

## Repeat-Path Loop Closure Scenario

Trajectory definition: `software/sim/config/loop_closure_path.json`

1. Fly Scout through the waypoints in order and return to start.
2. Capture evidence:

```bash
OUTPUT_DIR=output/rtabmap_loop_closure ./software/sim/tools/validate_loop_closure.sh
```

The loop-closure script now fails fast unless all of the following are true:
- RTAB-Map info stream contains loop-closure indicators
- OpenVINS odometry returns near the loop trajectory endpoint
- TF evidence exists for both `map -> odom` and `odom -> base_link`

3. Inspect artifacts:
- `output/rtabmap_loop_closure/rtabmap_info.log`
- `output/rtabmap_loop_closure/openvins_odom.log`
- `output/rtabmap_loop_closure/tf_map_odom.log`
- `output/rtabmap_loop_closure/tf_odom_base_link.log`

## Known Limitations

- Gazebo sensor topic names can vary between PX4 model builds; use `*_TOPIC_GZ` env overrides in `run_basic_sim.sh` when needed.
- Loop-closure confidence depends on scene texture and lighting; sparse features reduce closure frequency.
- Default queue/sync settings prioritize determinism over raw throughput and may need retuning for large maps.

## Tuning Knobs for Story 2.2

- `rtabmap_vio_sim.yaml`
  - `Grid/CellSize`
  - `Grid/RangeMax`
  - `RGBD/OptimizeMaxError`
  - `queue_size`
- `openvins_sim.yaml`
  - `max_synchronization_slop_sec`
  - `imu_rate_hz`
  - `stereo_baseline_m`

These values directly affect tile density, map correction smoothness, and first-usable-map latency.
