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
ros2 launch aeris_map rtabmap_vio_sim.launch.py
```

## Smoke Validation

```bash
SCOUT_NAME=scout1 ./software/sim/tools/validate_slam_topics.sh
```

## Loop-Closure Validation

```bash
OUTPUT_DIR=output/rtabmap_loop_closure ./software/sim/tools/validate_loop_closure.sh
```

## VIO Drift Validation (10-Minute Run)

```bash
python3 software/sim/tools/validate_vio_navigation_drift.py \
  --profile software/sim/config/vio_navigation_profile.json \
  --run-id latest
```
