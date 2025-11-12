#!/usr/bin/env bash
set -euo pipefail

WORLD_PATH=${WORLD_PATH:-software/sim/worlds/basic_world.sdf}
BRIDGE_TOPICS=${BRIDGE_TOPICS:-"/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"}

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "[run_basic_sim] ERROR: Source ROS 2 environment (e.g., source /opt/ros/humble/setup.bash)" >&2
  exit 1
fi

if [[ -z "${SIM_BIN:-}" ]]; then
  if command -v gz >/dev/null 2>&1; then
    SIM_BIN="gz sim"
    SIM_ARGS="-r -v 3"
  elif command -v ign >/dev/null 2>&1; then
    SIM_BIN="ign gazebo"
    SIM_ARGS="-r -s -v 3"
  else
    echo "[run_basic_sim] ERROR: Neither 'gz' nor 'ign' CLI found. Install Gazebo Sim." >&2
    exit 1
  fi
fi

echo "Launching Gazebo with world: ${WORLD_PATH} using ${SIM_BIN}"
${SIM_BIN} "${WORLD_PATH}" ${SIM_ARGS:-} &
GZ_PID=$!
trap 'kill ${GZ_PID} 2>/dev/null || true' EXIT

sleep 5

echo "Starting ros_gz_bridge parameter_bridge for topics: ${BRIDGE_TOPICS}"
ros2 run ros_gz_bridge parameter_bridge ${BRIDGE_TOPICS} &
BRIDGE_PID=$!
trap 'kill ${BRIDGE_PID} 2>/dev/null || true' EXIT

sleep 5
ros2 topic list | grep -q "/clock" && echo "Bridge active: /clock being published" || echo "WARNING: /clock not detected"

echo "Use a separate terminal to spawn the test model:"
echo "  ros2 service call /world/basic_world/create ros_gz_interfaces/srv/SpawnEntity '{name: test_box, xml: \"$(cat software/sim/models/test_box/model.sdf | sed 's/"/\\\"/g')\"}'"
echo "Then drive it (if plugin added) via ROS 2 topics."

wait ${GZ_PID}
