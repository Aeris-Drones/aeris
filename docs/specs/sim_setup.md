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
