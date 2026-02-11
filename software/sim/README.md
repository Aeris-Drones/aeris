# Simulation Scaffold

The `software/sim` tree hosts the Gazebo (Ignition/Fortress) assets for Aeris simulation environments. This package provides worlds, models, launch files, and validation tools for testing the Aeris autonomy stack in simulation.

## Directory Structure

```
software/sim/
├── config/           # Configuration files for multi-drone and recording
├── launch/           # ROS 2 launch files
├── models/           # Gazebo model definitions (SDF)
├── tools/            # Validation and utility scripts
└── worlds/           # Gazebo world files (SDF)
```

## Components

### Worlds

- **`worlds/aeris_block.world`**: Ground plane + sun with a static camera pole that publishes `/camera/image` for Phase 1 testing.
  - TODO: Add plume sources, thermal beacons, and street geometry

### Launch Files

- **`launch/sim_block.launch.py`**: ROS 2 launch file that runs `ign gazebo` (Fortress) with the world and starts `ros_gz_bridge parameter_bridge` for camera topics.

- **`launch/multi_drone_sim.launch.py`**: Multi-vehicle SITL bootstrap with:
  - Scout stereo/IMU bridge defaults for VIO/SLAM
  - Optional mission-node startup (`launch_orchestrator:=true` by default)
  - GPS-denied mode support

- **`launch/replay_recording.launch.py`**: Replay recorded simulation sessions for debugging and algorithm validation.

- **`launch/sim_world.launch.py`**: Generic world launcher with configurable world file and vehicle parameters.

### Configuration Files

- **`config/multi_drone.yaml`**: Multi-vehicle PX4 SITL configuration defining vehicle poses, MAVLink ports, and sensor bridge mappings.

- **`config/multi_drone_gps_denied.json`**: GPS-denied SITL profile with external-vision-oriented EKF defaults for scout vehicles.

- **`config/recording_profile.yaml`**: Rosbag2 recording configuration with MCAP storage and zstd compression.

- **`config/slam_parity.md`**: SLAM simulation parity specification documenting topic contracts, frame trees, and timing requirements.

- **`config/vio_navigation_profile.json`**: Shared GPS-denied launch + drift KPI profile for repeatable validation runs.

- **`config/loop_closure_path.json`**: Predefined trajectory waypoints for loop closure validation testing.

### Validation Tools

- **`tools/run_basic_sim.sh`**: Basic simulation runner with ROS-Gazebo bridge for stereo and IMU data. Supports both Ignition and Gazebo CLI with automatic detection.

- **`tools/validate_slam_topics.sh`**: Smoke validation for stereo/IMU/OpenVINS/RTAB-Map topic availability and publish rates.

- **`tools/validate_loop_closure.sh`**: Loop-closure evidence capture helper that analyzes `/rtabmap/info` and TF snapshots.

- **`tools/validate_vio_navigation_drift.py`**: Compares `/scout*/openvins/odom` against ground-truth odometry and writes RMS/max drift artifacts.

- **`tools/camera_path_builder.py`**: Interactive tool for designing camera inspection paths in simulation.

- **`tools/record_sim_session.py`**: Records simulation sessions with configurable profiles.

- **`tools/replay_mission.py`**: Replays recorded missions for regression testing.

- **`tools/run_multi_drone_sitl.py`**: Multi-drone SITL launcher with coordinated PX4 instances.

- **`tools/send_camera_view.py`**: Utility for sending camera view commands to simulation.

## Prerequisites

Inside the `osrf/ros:humble-desktop` container:

```bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge
```

## Usage Examples

### Basic Simulation

```bash
# From repo root inside osrf/ros:humble-desktop container
source /opt/ros/humble/setup.bash
colcon build --packages-select aeris_msgs aeris_map aeris_orchestrator
source install/setup.bash
ros2 launch software/sim/launch/sim_block.launch.py
```

### SLAM Integration

```bash
# Terminal 1: Launch SLAM nodes
source /workspace/aeris/install/setup.bash
ros2 launch aeris_map rtabmap_vio_sim.launch.py

# Terminal 2: Run topic validation
source /workspace/aeris/install/setup.bash
SCOUT_NAME=scout1 ./software/sim/tools/validate_slam_topics.sh

# Terminal 3: Test loop closure
source /workspace/aeris/install/setup.bash
OUTPUT_DIR=output/rtabmap_loop_closure ./software/sim/tools/validate_loop_closure.sh
```

### GPS-Denied Operation

```bash
source /workspace/aeris/install/setup.bash
ros2 launch software/sim/launch/multi_drone_sim.launch.py \
  world:=software/sim/worlds/disaster_scene.sdf \
  scout_model_name:=scout1 \
  vehicles_config:=software/sim/config/multi_drone_gps_denied.json \
  gps_denied_mode:=true \
  position_source_mode:=vio_odometry \
  launch_orchestrator:=true
```

### Drift KPI Validation

```bash
# 10-minute profile for VIO drift analysis
source /workspace/aeris/install/setup.bash
python3 software/sim/tools/validate_vio_navigation_drift.py \
  --profile software/sim/config/vio_navigation_profile.json \
  --run-id latest
```

### Recording and Replay

```bash
# Record a simulation session
RECORDING_PROFILE=default RECORD_DURATION=300 ./software/sim/tools/run_basic_sim.sh

# Replay for debugging
ros2 launch software/sim/launch/replay_recording.launch.py \
  bag_file:=log/recordings/aeris_mission_2024-01-15_10-30-00.mcap
```

## Environment Variables

### Simulation Control

| Variable | Description | Default |
|----------|-------------|---------|
| `WORLD_PATH` | Path to SDF world file | `software/sim/worlds/basic_world.sdf` |
| `SCOUT_MODEL_NAME` | Primary scout vehicle name | `scout1` |
| `GPS_DENIED_MODE` | Enable GPS-denied simulation | `false` |
| `POSITION_SOURCE_MODE` | Position source for navigation | `telemetry_geodetic` |

### Bridge Configuration

| Variable | Description | Default |
|----------|-------------|---------|
| `BRIDGE_TOPICS` | Custom bridge topic mappings | (default set) |
| `APPLY_BRIDGE_REMAPS_WITH_CUSTOM_TOPICS` | Apply remaps with custom topics | `0` |

### Recording

| Variable | Description |
|----------|-------------|
| `RECORDING_PROFILE` | Enable recording with specified profile |
| `RECORD_DURATION` | Recording duration in seconds |
| `RECORD_OUTPUT_DIR` | Output directory for recordings |
| `RECORD_PREFIX` | Filename prefix for recordings |

### Validation

| Variable | Description | Default |
|----------|-------------|---------|
| `SCOUT_NAME` | Vehicle name for validation | `scout1` |
| `TIMEOUT_SEC` | Topic discovery timeout | `45` |
| `MIN_CAMERA_HZ` | Minimum camera publish rate | `15` |
| `MIN_IMU_HZ` | Minimum IMU publish rate | `200` |
| `MIN_ODOM_HZ` | Minimum odometry publish rate | `15` |

## Validation Reference

### Topic Rate Thresholds

`validate_slam_topics.sh` enforces the following minimum publish rates:

- **Camera**: 15 Hz (stereo pair)
- **IMU**: 200 Hz
- **Odometry**: 15 Hz

### Loop Closure Evidence

`validate_loop_closure.sh` validates:

1. Loop-closure signal in `/rtabmap/info`
2. Odometry returns near trajectory endpoint
3. TF samples captured for `map -> odom` and `odom -> base_link`

## Notes

- When using custom `BRIDGE_TOPICS`, default stereo/IMU remaps are skipped. Set `APPLY_BRIDGE_REMAPS_WITH_CUSTOM_TOPICS=1` to enable remaps with custom topics.
- The multi-drone path provides stereo + IMU bridge wiring for Scout vehicles and launch/config scaffolding for OpenVINS + RTAB-Map.
- Tune exact Gazebo sensor topic names via `LEFT_*_TOPIC_GZ`, `RIGHT_*_TOPIC_GZ`, and `IMU_TOPIC_GZ` environment variables if your local PX4 model differs.

## Planned Enhancements

1. Add plume particles, thermal targets, and simple house/sidewalk meshes to `aeris_block.world`
2. Integrate RTAB-Map + MBTiles export for first-tile latency and `/map/tiles` KPI validation
3. Expand `ros_gz_bridge` coverage for depth, IMU, wind, gas, and control topics
4. Wire the sim launch into CI for headless KPI smoke tests (first tile latency, tile throughput, perception rates)
