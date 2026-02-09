#!/usr/bin/env bash
set -euo pipefail

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}

PIDS=()
trap cleanup EXIT

WORLD_PATH=${WORLD_PATH:-software/sim/worlds/basic_world.sdf}
WORLD_NAME=${WORLD_NAME:-$(basename "${WORLD_PATH}")}
WORLD_NAME=${WORLD_NAME%.*}
SCOUT_MODEL_NAME=${SCOUT_MODEL_NAME:-scout1}

SCRIPT_DIR=$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
MODEL_PATH="${REPO_ROOT}/software/sim/models"
PLUGIN_PATH="${REPO_ROOT}/install/aeris_camera_controller/lib"
RECORD_SCRIPT="${SCRIPT_DIR}/record_sim_session.py"

GZ_SCOUT_TOPIC_ROOT=${GZ_SCOUT_TOPIC_ROOT:-"/world/${WORLD_NAME}/model/${SCOUT_MODEL_NAME}"}

LEFT_IMAGE_TOPIC_GZ=${LEFT_IMAGE_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/stereo_left_link/sensor/stereo_left_camera/image"}
RIGHT_IMAGE_TOPIC_GZ=${RIGHT_IMAGE_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/stereo_right_link/sensor/stereo_right_camera/image"}
LEFT_INFO_TOPIC_GZ=${LEFT_INFO_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/stereo_left_link/sensor/stereo_left_camera/camera_info"}
RIGHT_INFO_TOPIC_GZ=${RIGHT_INFO_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/stereo_right_link/sensor/stereo_right_camera/camera_info"}
IMU_TOPIC_GZ=${IMU_TOPIC_GZ:-"${GZ_SCOUT_TOPIC_ROOT}/link/imu_link/sensor/imu_sensor/imu"}

LEFT_IMAGE_TOPIC_ROS=${LEFT_IMAGE_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/stereo/left/image_raw"}
RIGHT_IMAGE_TOPIC_ROS=${RIGHT_IMAGE_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/stereo/right/image_raw"}
LEFT_INFO_TOPIC_ROS=${LEFT_INFO_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/stereo/left/camera_info"}
RIGHT_INFO_TOPIC_ROS=${RIGHT_INFO_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/stereo/right/camera_info"}
IMU_TOPIC_ROS=${IMU_TOPIC_ROS:-"/${SCOUT_MODEL_NAME}/imu/data"}

if [[ -n "${BRIDGE_TOPICS:-}" ]]; then
  # shellcheck disable=SC2206
  BRIDGE_TOPIC_ARGS=(${BRIDGE_TOPICS})
else
  BRIDGE_TOPIC_ARGS=(
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
    "${LEFT_IMAGE_TOPIC_GZ}@sensor_msgs/msg/Image[gz.msgs.Image"
    "${RIGHT_IMAGE_TOPIC_GZ}@sensor_msgs/msg/Image[gz.msgs.Image"
    "${LEFT_INFO_TOPIC_GZ}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
    "${RIGHT_INFO_TOPIC_GZ}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
    "${IMU_TOPIC_GZ}@sensor_msgs/msg/Imu[gz.msgs.IMU"
  )
fi

BRIDGE_REMAP_ARGS=(
  "--ros-args"
  "-r" "${LEFT_IMAGE_TOPIC_GZ}:=${LEFT_IMAGE_TOPIC_ROS}"
  "-r" "${RIGHT_IMAGE_TOPIC_GZ}:=${RIGHT_IMAGE_TOPIC_ROS}"
  "-r" "${LEFT_INFO_TOPIC_GZ}:=${LEFT_INFO_TOPIC_ROS}"
  "-r" "${RIGHT_INFO_TOPIC_GZ}:=${RIGHT_INFO_TOPIC_ROS}"
  "-r" "${IMU_TOPIC_GZ}:=${IMU_TOPIC_ROS}"
)

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

if [[ -d "${PLUGIN_PATH}" ]]; then
  if [[ -z "${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}" ]]; then
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${PLUGIN_PATH}"
  else
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${PLUGIN_PATH}:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
  fi
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "[run_basic_sim] ERROR: Source ROS 2 environment (e.g., source /opt/ros/humble/setup.bash)" >&2
  exit 1
fi

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

echo "Launching Gazebo with world: ${WORLD_PATH} using ${SIM_BIN}"
echo "IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH:-}" >&2
echo "IGN_GAZEBO_SYSTEM_PLUGIN_PATH=${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}" >&2
${SIM_BIN} "${WORLD_PATH}" ${SIM_ARGS:-} &
GZ_PID=$!
PIDS+=("${GZ_PID}")

sleep 5

echo "Starting ros_gz_bridge parameter_bridge for topics:"
printf '  %s\n' "${BRIDGE_TOPIC_ARGS[@]}"
ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_TOPIC_ARGS[@]}" "${BRIDGE_REMAP_ARGS[@]}" &
BRIDGE_PID=$!
PIDS+=("${BRIDGE_PID}")

sleep 5
ros2 topic list | grep -q "/clock" && echo "Bridge active: /clock being published" || echo "WARNING: /clock not detected"

if [[ -n "${RECORDING_PROFILE:-}" ]]; then
  if [[ ! -f "${RECORD_SCRIPT}" ]]; then
    echo "[run_basic_sim] WARNING: RECORDING_PROFILE set but ${RECORD_SCRIPT} not found" >&2
  else
    echo "Starting recording via record_sim_session.py with profile ${RECORDING_PROFILE}"
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
    "${RECORD_CMD[@]}" &
    RECORD_PID=$!
    PIDS+=("${RECORD_PID}")
  fi
fi

echo "Use a separate terminal to spawn the test model:"
echo "  ros2 service call /world/${WORLD_NAME}/create ros_gz_interfaces/srv/SpawnEntity '{name: test_box, xml: \"$(cat software/sim/models/test_box/model.sdf | sed 's/"/\\\"/g')\"}'"
echo "Then drive it via ROS 2: bridge /model/test_box/cmd_vel and publish geometry_msgs/msg/Twist as documented in docs/specs/sim_setup.md."

wait ${GZ_PID}
