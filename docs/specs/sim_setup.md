# Simulation Setup

## Container Prerequisites

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select aeris_msgs aeris_map aeris_orchestrator --symlink-install
source install/setup.bash
```

## Start Disaster Scene + SITL

```bash
ros2 launch software/sim/launch/multi_drone_sim.launch.py \
  world:=software/sim/worlds/disaster_scene.sdf \
  scout_model_name:=scout1
```

## Start Disaster Scene + SITL (GPS-Denied VIO Mode)

```bash
ros2 launch software/sim/launch/multi_drone_sim.launch.py \
  world:=software/sim/worlds/disaster_scene.sdf \
  scout_model_name:=scout1 \
  vehicles_config:=software/sim/config/multi_drone_gps_denied.json \
  gps_denied_mode:=true \
  position_source_mode:=vio_odometry \
  launch_orchestrator:=true
```

## Start OpenVINS + RTAB-Map

```bash
ros2 launch aeris_map rtabmap_vio_sim.launch.py scout_model_name:=scout1
```

Optional per-run isolation:
- `openvins_log_directory:=/tmp/aeris/openvins/<scout_or_run_id>`
- `rtabmap_database_path:=/tmp/aeris/rtabmap/<scout_or_run_id>.db`

## Smoke Validation

```bash
SCOUT_NAME=scout1 ./software/sim/tools/validate_slam_topics.sh
```

Tune thresholds with:
- `MIN_CAMERA_HZ` (default `15`)
- `MIN_IMU_HZ` (default `200`)
- `MIN_ODOM_HZ` (default `15`)

## Loop-Closure Validation

```bash
OUTPUT_DIR=output/rtabmap_loop_closure ./software/sim/tools/validate_loop_closure.sh
```

This check now hard-fails when loop-closure tokens are missing, odometry does not return near the trajectory endpoint, or TF chain evidence is incomplete.

## VIO Drift Validation (10-Minute Run)

```bash
python3 software/sim/tools/validate_vio_navigation_drift.py \
  --profile software/sim/config/vio_navigation_profile.json \
  --run-id latest
```

## VIO Return-to-Launch Validation (GPS-Denied)

Trigger recall for an active scout:

```bash
ros2 service call /vehicle_command aeris_msgs/srv/VehicleCommand \
  "{command: RECALL, vehicle_id: scout1, mission_id: <ACTIVE_MISSION_ID>}"
```

Check canonical return telemetry extension:

```bash
ros2 topic echo /mission/progress --once
```

Required `returnTrajectory` keys:
- `vehicleId`
- `state`
- `points`
- `etaSec`
- `lastUpdatedSec`
- optional `fallbackReason`

Run smoke validation:

```bash
python3 software/edge/src/aeris_orchestrator/test/sitl_vio_return_smoke.py
```

Run fallback-path smoke validation (asserts `fallbackReason` emission):

```bash
EXPECT_FALLBACK=1 python3 software/edge/src/aeris_orchestrator/test/sitl_vio_return_smoke.py
```

Pass/fail thresholds:
- arrival horizontal error at launch point `<= 2.0 m`
- trajectory update latency `<= 1.0 s`
- fallback reason emitted on map/VIO freshness or synthesis failure
