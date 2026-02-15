#!/usr/bin/env bash
#
# run_basic_sim.sh - Basic Gazebo simulation runner for Aeris
#
# DESCRIPTION:
#   Launches a Gazebo simulation world with ROS-Gazebo bridge for stereo camera,
#   thermal camera, and IMU data. Supports both Ignition (Fortress) and Gazebo
#   (Harmonic/Ionic) CLI tools with automatic detection.
#
# USAGE:
#   ./run_basic_sim.sh [options]
#
# ENVIRONMENT VARIABLES:
#   WORLD_PATH              - Path to SDF world file (default: software/sim/worlds/basic_world.sdf)
#   WORLD_NAME              - World name override (default: derived from WORLD_PATH)
#   SCOUT_MODEL_NAME        - Name of the scout model (default: scout1)
#   GPS_DENIED_MODE         - Enable GPS-denied simulation mode (default: false)
#   POSITION_SOURCE_MODE    - Position source for GPS-denied mode (default: telemetry_geodetic)
#   VEHICLES_CONFIG         - Optional multi-vehicle JSON/YAML config for auto bridge topic generation
#   BRIDGE_TOPICS           - Custom bridge topic mappings (space-separated)
#   APPLY_BRIDGE_REMAPS_WITH_CUSTOM_TOPICS - Apply remaps with custom topics (default: 0)
#   RECORDING_PROFILE       - Enable recording with specified profile
#   RECORD_DURATION         - Recording duration in seconds
#   RECORD_OUTPUT_DIR       - Recording output directory
#   RECORD_PREFIX           - Recording file prefix
#   RECORD_TOPICS           - Additional topics to record
#   RECORD_DRY_RUN          - Dry run mode for recording (default: 0)
#
# EXAMPLES:
#   # Basic usage
#   ./run_basic_sim.sh
#
#   # GPS-denied mode with custom world
#   WORLD_PATH=worlds/custom.sdf GPS_DENIED_MODE=true ./run_basic_sim.sh
#
#   # With recording enabled
#   RECORDING_PROFILE=default RECORD_DURATION=300 ./run_basic_sim.sh
#

set -euo pipefail

# =============================================================================
# Cleanup and Signal Handling
# =============================================================================

# Cleanup function to terminate all background processes on exit
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}

# Array to track background process IDs
PIDS=()
trap cleanup EXIT

# =============================================================================
# Configuration and Environment Setup
# =============================================================================

# Simulation world configuration
WORLD_PATH=${WORLD_PATH:-software/sim/worlds/basic_world.sdf}
WORLD_NAME=${WORLD_NAME:-$(basename "${WORLD_PATH}")}
WORLD_NAME=${WORLD_NAME%.*}

# Vehicle configuration
SCOUT_MODEL_NAME=${SCOUT_MODEL_NAME:-scout1}
GPS_DENIED_MODE=${GPS_DENIED_MODE:-false}
POSITION_SOURCE_MODE=${POSITION_SOURCE_MODE:-telemetry_geodetic}
VEHICLES_CONFIG=${VEHICLES_CONFIG:-}

# Path resolution - determine repository root and resource paths
SCRIPT_DIR=$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/../../.." && pwd)
MODEL_PATH="${REPO_ROOT}/software/sim/models"
PLUGIN_PATH="${REPO_ROOT}/install/aeris_camera_controller/lib"
RECORD_SCRIPT="${SCRIPT_DIR}/record_sim_session.py"

# =============================================================================
# Gazebo Topic Configuration
# =============================================================================

# Gazebo topic root path for the scout model
GZ_SCOUT_TOPIC_ROOT=${GZ_SCOUT_TOPIC_ROOT:-"/world/${WORLD_NAME}/model/${SCOUT_MODEL_NAME}"}

# Gazebo sensor topics (source)
LEFT_IMAGE_TOPIC_GZ=${LEFT_IMAGE_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/stereo_left_link/sensor/stereo_left_camera/image"}
RIGHT_IMAGE_TOPIC_GZ=${RIGHT_IMAGE_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/stereo_right_link/sensor/stereo_right_camera/image"}
LEFT_INFO_TOPIC_GZ=${LEFT_INFO_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/stereo_left_link/sensor/stereo_left_camera/camera_info"}
RIGHT_INFO_TOPIC_GZ=${RIGHT_INFO_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/stereo_right_link/sensor/stereo_right_camera/camera_info"}
IMU_TOPIC_GZ=${IMU_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/imu_link/sensor/imu_sensor/imu"}
THERMAL_IMAGE_TOPIC_GZ=${THERMAL_IMAGE_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/thermal_link/sensor/thermal_camera/image"}

# =============================================================================
# ROS Topic Configuration
# =============================================================================

# ROS topic names (destination)
LEFT_IMAGE_TOPIC_ROS=${LEFT_IMAGE_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/stereo/left/image_raw"}
RIGHT_IMAGE_TOPIC_ROS=${RIGHT_IMAGE_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/stereo/right/image_raw"}
LEFT_INFO_TOPIC_ROS=${LEFT_INFO_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/stereo/left/camera_info"}
RIGHT_INFO_TOPIC_ROS=${RIGHT_INFO_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/stereo/right/camera_info"}
IMU_TOPIC_ROS=${IMU_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/imu/data"}
THERMAL_IMAGE_TOPIC_ROS=${THERMAL_IMAGE_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/thermal/image_raw"}

# =============================================================================
# Bridge Configuration
# =============================================================================

# Auto-generated bridge and remap arguments from multi-vehicle config
AUTO_BRIDGE_TOPIC_ARGS=()
AUTO_BRIDGE_REMAP_ARGS=()
if [[ -n "${VEHICLES_CONFIG}" ]]; then
  VEHICLES_CONFIG_PATH="${VEHICLES_CONFIG}"
  if [[ ! -f "${VEHICLES_CONFIG_PATH}" && -f "${REPO_ROOT}/${VEHICLES_CONFIG}" ]]; then
    VEHICLES_CONFIG_PATH="${REPO_ROOT}/${VEHICLES_CONFIG}"
  fi

  if [[ -f "${VEHICLES_CONFIG_PATH}" ]]; then
    while IFS= read -r line; do
      case "${line}" in
        TOPIC\ *)
          AUTO_BRIDGE_TOPIC_ARGS+=("${line#TOPIC }")
          ;;
        REMAP\ *)
          AUTO_BRIDGE_REMAP_ARGS+=("-r" "${line#REMAP }")
          ;;
      esac
    done < <(python3 - "${VEHICLES_CONFIG_PATH}" "${WORLD_NAME}" <<'PY'
from __future__ import annotations

import json
import sys
from pathlib import Path

config_path = Path(sys.argv[1])
world_name = str(sys.argv[2]).strip()
if not world_name:
    raise SystemExit(0)

text = config_path.read_text()
data = None
try:
    data = json.loads(text)
except json.JSONDecodeError:
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise SystemExit(f"PyYAML is required to parse {config_path}: {exc}") from exc
    data = yaml.safe_load(text)

if not isinstance(data, dict):
    raise SystemExit(0)
vehicles = data.get("vehicles")
if not isinstance(vehicles, list):
    raise SystemExit(0)

topic_specs = (
    (
        "left_image_topic",
        "sensor_msgs/msg/Image",
        "gz.msgs.Image",
        "link/stereo_left_link/sensor/stereo_left_camera/image",
    ),
    (
        "right_image_topic",
        "sensor_msgs/msg/Image",
        "gz.msgs.Image",
        "link/stereo_right_link/sensor/stereo_right_camera/image",
    ),
    (
        "thermal_image_topic",
        "sensor_msgs/msg/Image",
        "gz.msgs.Image",
        "link/thermal_link/sensor/thermal_camera/image",
    ),
    (
        "left_camera_info_topic",
        "sensor_msgs/msg/CameraInfo",
        "gz.msgs.CameraInfo",
        "link/stereo_left_link/sensor/stereo_left_camera/camera_info",
    ),
    (
        "right_camera_info_topic",
        "sensor_msgs/msg/CameraInfo",
        "gz.msgs.CameraInfo",
        "link/stereo_right_link/sensor/stereo_right_camera/camera_info",
    ),
    (
        "imu_topic",
        "sensor_msgs/msg/Imu",
        "gz.msgs.IMU",
        "link/imu_link/sensor/imu_sensor/imu",
    ),
)

clock_topic = "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
seen_topics = {clock_topic}
seen_remaps = set()
print(f"TOPIC {clock_topic}")

for vehicle in vehicles:
    if not isinstance(vehicle, dict):
        continue
    vehicle_name = str(vehicle.get("name", "")).strip()
    sensor_bridge = vehicle.get("sensor_bridge")
    if not vehicle_name or not isinstance(sensor_bridge, dict):
        continue
    topic_root = f"/world/{world_name}/model/{vehicle_name}"

    for key, ros_type, gz_type, topic_suffix in topic_specs:
        ros_topic = sensor_bridge.get(key)
        if not isinstance(ros_topic, str):
            continue
        ros_topic = ros_topic.strip()
        if not ros_topic:
            continue

        gz_topic = f"{topic_root}/{topic_suffix}"
        topic_arg = f"{gz_topic}@{ros_type}[{gz_type}"
        remap_arg = f"{gz_topic}:={ros_topic}"

        if topic_arg not in seen_topics:
            print(f"TOPIC {topic_arg}")
            seen_topics.add(topic_arg)
        if remap_arg not in seen_remaps:
            print(f"REMAP {remap_arg}")
            seen_remaps.add(remap_arg)
PY
    )

    if [[ ${#AUTO_BRIDGE_TOPIC_ARGS[@]} -gt 0 ]]; then
      echo "[run_basic_sim] INFO: auto-generated bridge topics from ${VEHICLES_CONFIG_PATH}" >&2
    fi
  else
    echo "[run_basic_sim] WARNING: VEHICLES_CONFIG not found: ${VEHICLES_CONFIG}" >&2
  fi
fi

# Build bridge topic arguments - maps Gazebo topics to ROS message types
if [[ -n "${BRIDGE_TOPICS:-}" ]]; then
  # Use custom bridge topics if provided
  IFS=' ' read -r -a BRIDGE_TOPIC_ARGS <<< "${BRIDGE_TOPICS}"
elif [[ ${#AUTO_BRIDGE_TOPIC_ARGS[@]} -gt 0 ]]; then
  BRIDGE_TOPIC_ARGS=("${AUTO_BRIDGE_TOPIC_ARGS[@]}")
else
  # Default bridge configuration for stereo vision, thermal, and IMU
  BRIDGE_TOPIC_ARGS=(
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
    "${LEFT_IMAGE_TOPIC_GZ}@sensor_msgs/msg/Image[gz.msgs.Image"
    "${RIGHT_IMAGE_TOPIC_GZ}@sensor_msgs/msg/Image[gz.msgs.Image"
    "${THERMAL_IMAGE_TOPIC_GZ}@sensor_msgs/msg/Image[gz.msgs.Image"
    "${LEFT_INFO_TOPIC_GZ}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
    "${RIGHT_INFO_TOPIC_GZ}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
    "${IMU_TOPIC_GZ}@sensor_msgs/msg/Imu[gz.msgs.IMU"
  )
fi

# Configure topic remapping from Gazebo to ROS topic names
if [[ -n "${BRIDGE_TOPICS:-}" ]]; then
  if [[ "${APPLY_BRIDGE_REMAPS_WITH_CUSTOM_TOPICS:-0}" == "1" ]]; then
    BRIDGE_REMAP_ARGS=(
      "--ros-args"
      "-r" "${LEFT_IMAGE_TOPIC_GZ}:=${LEFT_IMAGE_TOPIC_ROS}"
      "-r" "${RIGHT_IMAGE_TOPIC_GZ}:=${RIGHT_IMAGE_TOPIC_ROS}"
      "-r" "${THERMAL_IMAGE_TOPIC_GZ}:=${THERMAL_IMAGE_TOPIC_ROS}"
      "-r" "${LEFT_INFO_TOPIC_GZ}:=${LEFT_INFO_TOPIC_ROS}"
      "-r" "${RIGHT_INFO_TOPIC_GZ}:=${RIGHT_INFO_TOPIC_ROS}"
      "-r" "${IMU_TOPIC_GZ}:=${IMU_TOPIC_ROS}"
    )
  else
    BRIDGE_REMAP_ARGS=()
    echo "[run_basic_sim] INFO: custom BRIDGE_TOPICS detected; default stereo/thermal/IMU remaps skipped." >&2
    echo "[run_basic_sim] INFO: set APPLY_BRIDGE_REMAPS_WITH_CUSTOM_TOPICS=1 or provide matching remap vars to enable remaps." >&2
  fi
elif [[ ${#AUTO_BRIDGE_REMAP_ARGS[@]} -gt 0 ]]; then
  BRIDGE_REMAP_ARGS=("--ros-args" "${AUTO_BRIDGE_REMAP_ARGS[@]}")
else
  BRIDGE_REMAP_ARGS=(
    "--ros-args"
    "-r" "${LEFT_IMAGE_TOPIC_GZ}:=${LEFT_IMAGE_TOPIC_ROS}"
    "-r" "${RIGHT_IMAGE_TOPIC_GZ}:=${RIGHT_IMAGE_TOPIC_ROS}"
    "-r" "${THERMAL_IMAGE_TOPIC_GZ}:=${THERMAL_IMAGE_TOPIC_ROS}"
    "-r" "${LEFT_INFO_TOPIC_GZ}:=${LEFT_INFO_TOPIC_ROS}"
    "-r" "${RIGHT_INFO_TOPIC_GZ}:=${RIGHT_INFO_TOPIC_ROS}"
    "-r" "${IMU_TOPIC_GZ}:=${IMU_TOPIC_ROS}"
  )
fi

# =============================================================================
# Resource Path Configuration
# =============================================================================

# Add model path to Gazebo resource paths for both Ignition and Gazebo
if [[ -d "${MODEL_PATH}" ]]; then
  if [[ -z "${IGN_GAZEBO_RESOURCE_PATH:-}" ]]; then
    export IGN_GAZEBO_RESOURCE_PATH="${MODEL_PATH}"
  else
    export IGN_GAZEBO_RESOURCE_PATH="${MODEL_PATH}:${IGN_GAZEBO_RESOURCE_PATH}"
  fi
  if [[ -z "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
    export GZ_SIM_RESOURCE_PATH="${MODEL_PATH}"
  else
    export GZ_SIM_RESOURCE_PATH="${MODEL_PATH}:${GZ_SIM_RESOURCE_PATH}"
  fi
fi

# Add plugin path to Gazebo system plugin path
if [[ -d "${PLUGIN_PATH}" ]]; then
  if [[ -z "${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}" ]]; then
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${PLUGIN_PATH}"
  else
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${PLUGIN_PATH}:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
  fi
fi

# =============================================================================
# Environment Validation
# =============================================================================

# Verify ROS 2 environment is sourced
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "[run_basic_sim] ERROR: Source ROS 2 environment (e.g., source /opt/ros/humble/setup.bash)" >&2
  exit 1
fi

# Auto-detect Gazebo/Ignition CLI and set appropriate arguments
if [[ -z "${SIM_BIN:-}" ]]; then
  if command -v ign >/dev/null 2>&1; then
    SIM_BIN="ign gazebo"
    SIM_ARGS="-r -s -v 3"
  elif command -v gz >/dev/null 2>&1; then
    SIM_BIN="gz sim"
    SIM_ARGS="-r -v 3"
  else
    echo "[run_basic_sim] ERROR: Neither 'ign' nor 'gz' CLI found. Install Gazebo Sim." >&2
    exit 1
  fi
fi

# =============================================================================
# Launch Gazebo Simulation
# =============================================================================

echo "Launching Gazebo with world: ${WORLD_PATH} using ${SIM_BIN}"
echo "GPS-denied mode: ${GPS_DENIED_MODE} (position source: ${POSITION_SOURCE_MODE})"
echo "IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH:-}" >&2
echo "IGN_GAZEBO_SYSTEM_PLUGIN_PATH=${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}" >&2

# Start Gazebo in background
${SIM_BIN} "${WORLD_PATH}" ${SIM_ARGS:-} &
GZ_PID=$!
PIDS+=("${GZ_PID}")

# Allow Gazebo time to initialize
sleep 5

# =============================================================================
# Launch ROS-Gazebo Bridge
# =============================================================================

echo "Starting ros_gz_bridge parameter_bridge for topics:"
printf '  %s\n' "${BRIDGE_TOPIC_ARGS[@]}"

ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_TOPIC_ARGS[@]}" "${BRIDGE_REMAP_ARGS[@]}" &
BRIDGE_PID=$!
PIDS+=("${BRIDGE_PID}")

# Allow bridge time to establish connections
sleep 5

# Verify bridge is active by checking for /clock topic
ros2 topic list | grep -q "/clock" && echo "Bridge active: /clock being published" || echo "WARNING: /clock not detected"

# =============================================================================
# Optional Recording Session
# =============================================================================

if [[ -n "${RECORDING_PROFILE:-}" ]]; then
  if [[ ! -f "${RECORD_SCRIPT}" ]]; then
    echo "[run_basic_sim] WARNING: RECORDING_PROFILE set but ${RECORD_SCRIPT} not found" >&2
  else
    echo "Starting recording via record_sim_session.py with profile ${RECORDING_PROFILE}"

    # Build recording command with optional parameters
    RECORD_CMD=(python3 "${RECORD_SCRIPT}" --config "${RECORDING_PROFILE}")

    if [[ -n "${RECORD_DURATION:-}" ]]; then
      RECORD_CMD+=("--duration" "${RECORD_DURATION}")
    fi
    if [[ -n "${RECORD_OUTPUT_DIR:-}" ]]; then
      RECORD_CMD+=("--output-dir" "${RECORD_OUTPUT_DIR}")
    fi
    if [[ -n "${RECORD_PREFIX:-}" ]]; then
      RECORD_CMD+=("--prefix" "${RECORD_PREFIX}")
    fi
    if [[ -n "${RECORD_TOPICS:-}" ]]; then
      # shellcheck disable=SC2206
      CUSTOM_TOPICS=(${RECORD_TOPICS})
      if [[ ${#CUSTOM_TOPICS[@]} -gt 0 ]]; then
        RECORD_CMD+=("--topics")
        RECORD_CMD+=("${CUSTOM_TOPICS[@]}")
      fi
    fi
    if [[ "${RECORD_DRY_RUN:-0}" == "1" ]]; then
      RECORD_CMD+=("--dry-run")
    fi

    # Start recording in background
    "${RECORD_CMD[@]}" &
    RECORD_PID=$!
    PIDS+=("${RECORD_PID}")
  fi
fi

# =============================================================================
# User Instructions
# =============================================================================

echo "Use a separate terminal to spawn the test model:"
echo "  ros2 service call /world/${WORLD_NAME}/create ros_gz_interfaces/srv/SpawnEntity '{name: test_box, xml: \"$(cat software/sim/models/test_box/model.sdf | sed 's/\"/\\\"/g')\"}'"
echo "Then drive it via ROS 2: bridge /model/test_box/cmd_vel and publish geometry_msgs/msg/Twist as documented in docs/specs/sim_setup.md."

# Wait for Gazebo process to complete
wait ${GZ_PID}
