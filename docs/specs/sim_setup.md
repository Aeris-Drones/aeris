# Simulation Setup Guide (Draft)

This guide outlines steps to prepare Gazebo Sim with ROS 2 Humble integration and a basic workspace for Aeris simulation development.

## Prerequisites
- Ubuntu 22.04 recommended (ROS 2 Humble target)
- Docker optional for isolation

## Install Overview (Verify per environment)
> Note: Exact package names and repo steps must be verified against official documentation. If you need me to fetch and pin commands with citations, ask for a research pass.

1. Install ROS 2 Humble (desktop) and source setup
2. Install Gazebo Sim (Fortress/Harmonic) and ros-gz-bridge compatible with Humble
3. Install PX4 SITL (optional for later multi-vehicle tasks)

## Workspace Layout
```
software/sim/
  launch/
  models/
  worlds/
```

## Build Commands
Compile the cinematic camera plugin and ROS interfaces before running Gazebo:
```bash
colcon build --packages-select aeris_msgs aeris_map aeris_orchestrator aeris_camera_controller --symlink-install
source install/setup.bash
```
`run_basic_sim.sh` now exports `IGN_GAZEBO_RESOURCE_PATH`, `GZ_SIM_RESOURCE_PATH`, and `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` so `model://cinematic_camera` plus `libaeris_camera_controller.so` load automatically.

## Quick Start
1. Build or obtain a basic world file under `software/sim/worlds/basic_world.sdf` (already scaffolded)
2. Launch the provided helper script inside the ROS 2 container (auto-detects `gz sim` vs `ign gazebo`):
   ```bash
   ./software/sim/tools/run_basic_sim.sh
   ```
3. In a separate terminal (with ROS sourced), spawn the `test_box` model using the `ros_gz_interfaces/srv/SpawnEntity` service.
4. Confirm `/clock` (and any additional bridged topics) appear via `ros2 topic list`.
5. Publish commands to the model (e.g., `/model/test_box/cmd_vel`) once the appropriate plugin is added.

### Disaster Scene Variant
Use the same helper with a different world path:
```bash
WORLD_PATH=software/sim/worlds/disaster_scene.sdf ./software/sim/tools/run_basic_sim.sh
```
See `docs/specs/disaster_scene.md` for lighting/layout details.

### Test Model Control Check
1. Bridge the command/odom topics (after the helper script launches Gazebo):
   ```bash
   ros2 run ros_gz_bridge parameter_bridge \
     /model/test_box/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
     /model/test_box/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry
   ```
2. Command the model from ROS 2:
   ```bash
   ros2 topic pub /model/test_box/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
   ```
3. Observe the robot driving in Gazebo and odom streaming back through the bridge.

### Multi-Drone SITL
1. Launch the disaster scene as above.
2. Validate ports/poses:
   ```bash
   ./software/sim/tools/run_multi_drone_sitl.py --config software/sim/config/multi_drone.yaml --dry-run
   ```
3. After installing PX4 SITL, spawn vehicles:
   ```bash
   ./software/sim/tools/run_multi_drone_sitl.py --config software/sim/config/multi_drone.yaml --execute
   ```
4. Optional: use `ros2 launch software/sim/launch/multi_drone_sim.launch.py` to orchestrate Gazebo + SITL in one go.
5. Inspect `/vehicle/<name>/...` topics via `ros2 topic list` once bridges are active.

### Single-Vehicle PX4 SITL Smoke Test
Epic 1 expects the PX4 toolchain to be verified before multi-vehicle work:
```bash
./software/sim/tools/run_multi_drone_sitl.py --config software/sim/config/multi_drone.yaml --vehicles scout1 --execute
```
This launches only the Scout1 instance (port 14540). Use `ros2 topic echo /vehicle/scout1/px4/vehicle_status` (after starting ros_gz_bridge and PX4-required bridges) to confirm heartbeats.

## Cinematic Camera Controller

- `software/edge/src/aeris_camera_controller`: Gazebo plugin exposing ROS 2 service `/simulation/set_camera_view`.
- `software/sim/models/cinematic_camera`: rig instantiated inside `disaster_scene.sdf`.
- Presets: `wide`, `tracking`, `pov`, `orbit`. Tracking/orbit keep following targets with cubic Bezier easing; wide shots ease for ~3 s by default.

### Launch Environment
- `software/sim/launch/disaster_scene.launch.py` sets resource + plugin paths; pass `auto_launch:=true` to spawn Ignition directly:
  ```bash
  ros2 launch software/sim/launch/disaster_scene.launch.py auto_launch:=true
  ```
- `run_basic_sim.sh` inherits the same env vars, so `WORLD_PATH=software/sim/worlds/disaster_scene.sdf ./software/sim/tools/run_basic_sim.sh` loads the camera rig without manual exports.

### Service Usage
1. Confirm the service is online: `ros2 service list | grep set_camera_view`.
2. Trigger a preset:
   ```bash
   python3 software/sim/tools/send_camera_view.py --preset tracking --target scout1 --track
   ```
3. Script a path: generate JSON and send it.
   ```bash
   ./software/sim/tools/camera_path_builder.py --template --output /tmp/hero_pan.json
   python3 software/sim/tools/send_camera_view.py --path-json /tmp/hero_pan.json --transition 8.0
   ```

`camera_path_builder.py` also accepts ad-hoc keyframes via `--keyframe time,x,y,z,roll,pitch,yaw,fov`. The sender script converts JSON entries into `aeris_msgs/srv/SetCameraView` requests and waits for the plugin to acknowledge.

### Recording & Replay (Story 1.5)
Create compressed rosbag recordings and replay them with cinematic control:

See `docs/specs/sim_recording.md` for the full scope/architecture.

1. **Configure topics:** Edit `software/sim/config/recording_profile.yaml` to toggle topics (default includes `/clock`, `/tf`, PX4 vehicle states, perception feeds, and camera commands).
2. **Capture a session:**
   ```bash
   python3 software/sim/tools/record_sim_session.py \
     --config software/sim/config/recording_profile.yaml \
     --duration 300
   ```
   - Creates `log/recordings/<timestamp>_<prefix>/` with MCAP+zstd recording plus `metadata.json`.
   - Use `RECORDING_PROFILE=software/sim/config/recording_profile.yaml ./software/sim/tools/run_basic_sim.sh` to auto-start recording alongside Gazebo (optional env vars: `RECORD_DURATION`, `RECORD_TOPICS`, `RECORD_OUTPUT_DIR`, `RECORD_DRY_RUN=1`).
3. **Replay the mission:**
   ```bash
   python3 software/sim/tools/replay_mission.py \
     --bag log/recordings/20251112T120501_aeris_mission/aeris_mission \
     --launch-world \
     --camera-preset orbit --camera-target ranger1
   ```
   - Launches `run_basic_sim.sh` headless (unless `--launch-world` omitted), waits for `/simulation/set_camera_view`, then runs `ros2 bag play`.
   - Use `software/sim/launch/replay_recording.launch.py bag:=...` for an equivalent launch-based workflow (supports `loop`, `world_path`, `headless` arguments).
4. **Budget checks:** The recorder warns if total size exceeds the 500 MB/5 minute target. Move heavy sessions to archival storage under `log/recordings/archive/`.

## Troubleshooting
- Confirm environment variables are sourced: `source /opt/ros/humble/setup.bash`
- Check bridge compatibility: versions of gz-sim and ros-gz-bridge
- If Gazebo fails to launch, validate GPU drivers (or run with `LIBGL_ALWAYS_SOFTWARE=1` for headless)
- Use `gz topic` to introspect Gazebo transport when ROS bridge is silent
