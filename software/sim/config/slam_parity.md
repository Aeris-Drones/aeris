# SLAM Simulation Parity Parameters

This document defines the simulation-specific configuration and topic contracts for the Aeris SLAM stack (OpenVINS + RTAB-Map). These parameters ensure consistent behavior between simulation and hardware deployments.

## Overview

The SLAM simulation parity specification establishes:
- Topic naming conventions for sensor data
- TF frame hierarchy and relationships
- Timing and synchronization requirements
- Publish rate thresholds for real-time performance

## Topic Contract (ROS 2)

The following topics must be available for SLAM operation:

### Sensor Input Topics

| Topic | Message Type | Description | Rate |
|-------|--------------|-------------|------|
| `/scout1/stereo/left/image_raw` | `sensor_msgs/Image` | Left stereo camera images | >= 15 Hz |
| `/scout1/stereo/right/image_raw` | `sensor_msgs/Image` | Right stereo camera images | >= 15 Hz |
| `/scout1/stereo/left/camera_info` | `sensor_msgs/CameraInfo` | Left camera calibration | >= 15 Hz |
| `/scout1/stereo/right/camera_info` | `sensor_msgs/CameraInfo` | Right camera calibration | >= 15 Hz |
| `/scout1/imu/data` | `sensor_msgs/Imu` | IMU measurements | >= 200 Hz |
| `/scout1/openvins/odom` | `nav_msgs/Odometry` | Visual-inertial odometry output | >= 15 Hz |

### Mapping Output Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | 2D occupancy grid map |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | 3D point cloud map |
| `/rtabmap/info` | `rtabmap_ros/Info` | RTAB-Map statistics and loop closure events |

## Frame Tree Contract

The TF tree follows the standard ROS navigation hierarchy:

```
map
└── odom
    └── base_link
        ├── stereo_left_optical_frame
        ├── stereo_right_optical_frame
        └── imu_link
```

### Frame Definitions

| Frame | Parent | Description |
|-------|--------|-------------|
| `map` | (world) | Global map coordinate frame |
| `odom` | `map` | Odometry origin (drifts relative to map) |
| `base_link` | `odom` | Robot body center |
| `stereo_left_optical_frame` | `base_link` | Left camera optical center |
| `stereo_right_optical_frame` | `base_link` | Right camera optical center |
| `imu_link` | `base_link` | IMU sensor location |

## Synchronization and Timing Requirements

### Timestamp Alignment

- Stereo camera pairs must be hardware-synchronized or timestamp-aligned
- Maximum timestamp slop between stereo and IMU: **10 milliseconds**
- All sensors must use `use_sim_time=true` in simulation

### Publish Rate Targets

| Sensor | Minimum Rate | Target Rate |
|--------|--------------|-------------|
| Stereo cameras | 15 Hz | 20 Hz |
| IMU | 200 Hz | 250 Hz |
| Odometry | 15 Hz | 30 Hz |

### Simulation Time

When running in simulation:
```bash
# Ensure all SLAM nodes use simulation time
ros2 param set /openvins use_sim_time true
ros2 param set /rtabmap use_sim_time true
```

## Configuration Sources

### OpenVINS Configuration
- **File**: `software/edge/src/aeris_map/config/openvins_sim.yaml`
- **Purpose**: Visual-inertial odometry parameters for simulation
- **Key parameters**: Feature tracking, IMU noise model, calibration

### RTAB-Map Configuration
- **File**: `software/edge/src/aeris_map/config/rtabmap_vio_sim.yaml`
- **Purpose**: SLAM and loop closure parameters
- **Key parameters**: VIO integration, loop closure detection, map optimization

### Launch File
- **File**: `software/edge/src/aeris_map/launch/rtabmap_vio_sim.launch.py`
- **Purpose**: Coordinated launch of OpenVINS + RTAB-Map with simulation overrides

## Architecture Guardrails

### Core Processing Nodes

No simulation-specific branches are permitted in core SLAM processing nodes. All environment-specific differences must be isolated to:

- Launch files (`*.launch.py`)
- Configuration files (`*.yaml`)
- Simulation scripts (`software/sim/tools/*.sh`)

### Validation

Use the provided validation scripts to verify SLAM parity:

```bash
# Verify topic availability and rates
SCOUT_NAME=scout1 ./software/sim/tools/validate_slam_topics.sh

# Validate loop closure detection
OUTPUT_DIR=output/rtabmap_loop_closure ./software/sim/tools/validate_loop_closure.sh
```

## References

- [OpenVINS Documentation](https://docs.openvins.com/)
- [RTAB-Map ROS Wiki](http://wiki.ros.org/rtabmap_ros)
- [ROS 2 Navigation TF Conventions](https://ros.org/reps/rep-0105.html)
