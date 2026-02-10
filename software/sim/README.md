# Simulation Scaffold

The `software/sim` tree hosts the Gazebo (Ignition/Fortress) assets used during the Conrad Innovation run-up.

## Current components
- `worlds/aeris_block.world`: ground plane + sun with a static camera pole that publishes `/camera/image` for Phase 1 testing. TODO markers remain for plume sources, thermal beacons, and street geometry.
- `launch/sim_block.launch.py`: ROS 2 launch file that runs `ign gazebo` (Fortress) with the world and starts `ros_gz_bridge parameter_bridge /camera/image@sensor_msgs/msg/Image@ignition.msgs.Image`.
- `launch/multi_drone_sim.launch.py`: multi-vehicle SITL bootstrap that now includes Scout stereo/IMU bridge defaults for VIO/SLAM.
- `config/multi_drone_gps_denied.yaml`: GPS-denied SITL profile with external-vision-oriented EKF defaults for scout vehicles.
- `config/vio_navigation_profile.yaml`: shared GPS-denied launch + drift KPI profile for repeatable validation runs.
- `tools/validate_slam_topics.sh`: smoke validation for stereo/IMU/OpenVINS/RTAB-Map topic availability and rates.
- `tools/validate_loop_closure.sh`: loop-closure evidence capture helper (`/rtabmap/info` + TF snapshot).
- `tools/validate_vio_navigation_drift.py`: compares `/scout*/openvins/odom` against ground-truth odometry and writes RMS/max drift artifacts.

## Prerequisites (inside osrf/ros:humble-desktop)
```
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge
```

## Running inside the ROS 2 Docker container
```
# repo root inside osrf/ros:humble-desktop
source /opt/ros/humble/setup.bash
colcon build --packages-select aeris_msgs aeris_map aeris_orchestrator
source install/setup.bash
ros2 launch software/sim/launch/sim_block.launch.py

# separate shells
source /workspace/aeris/install/setup.bash
ros2 run aeris_map map_tile_publisher

source /workspace/aeris/install/setup.bash
python3 software/edge/tools/first_tile_timer.py --timeout-sec 120

# Story 2.1 SLAM integration launch
source /workspace/aeris/install/setup.bash
ros2 launch aeris_map rtabmap_vio_sim.launch.py

# Smoke checks for topic activity and frame-chain inputs
source /workspace/aeris/install/setup.bash
SCOUT_NAME=scout1 ./software/sim/tools/validate_slam_topics.sh

# Loop-closure evidence capture (repeat-path runbook)
source /workspace/aeris/install/setup.bash
OUTPUT_DIR=output/rtabmap_loop_closure ./software/sim/tools/validate_loop_closure.sh

# GPS-denied launch profile (Story 2.3)
source /workspace/aeris/install/setup.bash
ros2 launch software/sim/launch/multi_drone_sim.launch.py \
  world:=software/sim/worlds/disaster_scene.sdf \
  scout_model_name:=scout1 \
  vehicles_config:=software/sim/config/multi_drone_gps_denied.yaml \
  gps_denied_mode:=true \
  position_source_mode:=vio_odometry

# Drift KPI evidence (10-minute profile)
source /workspace/aeris/install/setup.bash
python3 software/sim/tools/validate_vio_navigation_drift.py \
  --profile software/sim/config/vio_navigation_profile.yaml \
  --run-id latest
```

> The multi-drone path now provides stereo + IMU bridge wiring for Scout and launch/config scaffolding for OpenVINS + RTAB-Map. Tune exact Gazebo sensor topic names via `LEFT_*_TOPIC_GZ`, `RIGHT_*_TOPIC_GZ`, and `IMU_TOPIC_GZ` env overrides if your local PX4 model differs.

### Planned next steps
1. Add plume particles, thermal targets, and simple house/sidewalk meshes to `aeris_block.world`.
2. Integrate RTAB-Map + MBTiles export so the sim path can prove first-tile latency and `/map/tiles` KPIs before hardware exists.
3. Expand `ros_gz_bridge` coverage for depth, IMU, wind, gas, and control topics.
4. Wire the sim launch into CI to run headless KPI smoke tests (first tile latency, tile throughput, perception rates).
