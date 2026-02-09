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
