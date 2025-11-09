# Simulation Scaffold

The `software/sim` tree hosts the Gazebo (Ignition/Fortress) assets used during the Conrad Innovation run-up.

## Current components
- `worlds/aeris_block.world`: ground plane + sun with a static camera pole that publishes `/camera/image` for Phase 1 testing. TODO markers remain for plume sources, thermal beacons, and street geometry.
- `launch/sim_block.launch.py`: ROS 2 launch file that runs `ign gazebo` (Fortress) with the world and starts `ros_gz_bridge parameter_bridge /camera/image@sensor_msgs/msg/Image@ignition.msgs.Image`.

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
```

> Gazebo currently shows a minimal ground plane with the camera pole. Upcoming work will drop in plume emitters, thermal targets, RTAB-Map, and the ros_gz_bridge depth/IMU topics.

### Planned next steps
1. Add plume particles, thermal targets, and simple house/sidewalk meshes to `aeris_block.world`.
2. Integrate RTAB-Map + MBTiles export so the sim path can prove first-tile latency and `/map/tiles` KPIs before hardware exists.
3. Expand `ros_gz_bridge` coverage for depth, IMU, wind, gas, and control topics.
4. Wire the sim launch into CI to run headless KPI smoke tests (first tile latency, tile throughput, perception rates).
