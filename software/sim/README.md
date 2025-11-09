# Simulation Scaffold

The `software/sim` tree hosts the Gazebo (Ignition/Fortress) assets used during the Conrad Innovation run-up.

## Current components
- `worlds/aeris_block.world`: minimal ground-plane world with TODO markers for plume sources, thermal beacons, and street geometry.
- `launch/sim_block.launch.py`: ROS 2 launch file that runs `ign gazebo` with the world and prints a placeholder note where `ros_gz_bridge` topic mappings will live.

## Running inside the ROS 2 Docker container
```
# from the repo root inside osrf/ros:humble-desktop
source /opt/ros/humble/setup.bash
colcon build --packages-select aeris_msgs aeris_map aeris_orchestrator
source install/setup.bash
ros2 launch software/sim/launch/sim_block.launch.py
```

> Gazebo will currently show a blank ground plane. Phase 1 will populate the world with the street block, plume emitters, and integrate RTAB-Map + ros_gz_bridge topic flow.

### Planned next steps
1. Add plume particles, thermal targets, and simple house meshes to `aeris_block.world`.
2. Integrate RTAB-Map + MBTiles export so the sim path can prove first-tile latency and `/map/tiles` KPIs before hardware exists.
3. Replace the `LogInfo` stub with real `ros_gz_bridge` processes for cameras, depth, IMU, wind, gas, and command topics.
4. Wire the sim launch into CI to run headless KPI smoke tests (first tile latency, tile throughput, perception rates).
