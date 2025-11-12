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

## Troubleshooting
- Confirm environment variables are sourced: `source /opt/ros/humble/setup.bash`
- Check bridge compatibility: versions of gz-sim and ros-gz-bridge
- If Gazebo fails to launch, validate GPU drivers (or run with `LIBGL_ALWAYS_SOFTWARE=1` for headless)
- Use `gz topic` to introspect Gazebo transport when ROS bridge is silent
