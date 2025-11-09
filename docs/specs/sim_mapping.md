# Simulation Mapping Baseline (Phase 1)

This guide documents the minimum workflow required to produce simulated `/map/tiles` traffic with Gazebo (Ignition Fortress), bridge a camera topic into ROS 2, and measure "time to first tile".

## Prerequisites

Inside the ROS 2 Humble Docker container (`osrf/ros:humble-desktop`):

```bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge
```

`ros-humble-ros-gz` installs the Fortress-based Gazebo binaries plus integration packages, and `ros-humble-ros-gz-bridge` provides the `parameter_bridge` utility that maps Gazebo Transport topics into ROS 2 (`docs.ros.org` ros_gz_bridge reference).

## Launch workflow

1. Source ROS 2 and build the current packages:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select aeris_msgs aeris_map aeris_orchestrator
   source install/setup.bash
   ```
2. Start the simulator + bridge:
   ```bash
   ros2 launch software/sim/launch/sim_block.launch.py
   ```
   - The launch file runs `ign gazebo -r <world>` (Fortress default) per the Gazebo Fortress manual.
   - A `ros2 run ros_gz_bridge parameter_bridge /camera/image@sensor_msgs/msg/Image@ignition.msgs.Image` process forwards the Gazebo Transport camera stream onto ROS 2.
3. In a second shell (same workspace overlay) run the publisher:
   ```bash
   ros2 run aeris_map map_tile_publisher
   ```
4. In a third shell start the timing utility:
   ```bash
   python3 software/edge/tools/first_tile_timer.py --timeout-sec 120
   ```
   The utility prints `First tile in X.XXX seconds` once `/map/tiles` is observed and exits with status 0. It times out (non-zero exit) after 120 s unless overridden.

## Acceptance criteria

- Simulator + bridge launch without errors, with Gazebo showing the camera pole scene.
- `ros2 run aeris_map map_tile_publisher` produces synthetic tiles while the sim runs.
- `first_tile_timer.py` reports `First tile in X.XXX seconds` in under 90 s.
- Logs (or screenshots) from all three shells are attached to the PR/issue for traceability.

## Measured results

- First tile timings (Humble container): 0.681 s when the publisher started first, plus 2.26 s and 2.24 s when starting the timer and publisher nearly simultaneously via `python3 software/edge/tools/first_tile_timer.py --timeout-sec 120 & sleep 0.3; ros2 run aeris_map map_tile_publisher`.
- The earlier ~31 s data point captured the timer waiting long before the publisher spawned, so the delta included idle time rather than network latency.

## Troubleshooting (Headless)

- Run Fortress with `ign gazebo -r <world>` (render headless) and add `-s` to skip physics GUI initialization.
- If the GUI still attempts to start inside Docker, export `QT_QPA_PLATFORM=offscreen` before launching to avoid X11 errors.

## Roadmap

- Phase 2: add RTAB-Map + MBTiles writer so the simulated run hits the navigation KPIs before hardware is ready.
- Phase 3: introduce plume particles, thermal beacons, and ros_gz_bridge depth/IMU topics per the perception roadmap.

## References

1. ROS 2 Humble ros_gz apt packages (`ros-humble-ros-gz`, `ros-humble-ros-gz-bridge`) – https://docs.ros.org/en/humble/p/ros_gz_bridge/
2. Ignition/Gazebo SDF camera sensor specification and topic naming – http://sdformat.org/spec?elem=sensor
3. ros_gz_bridge `parameter_bridge` CLI syntax (`/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image`) – https://gazebosim.org/docs/latest/ros2_integration/
4. Gazebo Fortress runtime command (`ign gazebo`) – https://gazebosim.org/docs/fortress/getstarted/
