#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SCOUT_NAME=${SCOUT_NAME:-scout1}
TIMEOUT_SEC=${TIMEOUT_SEC:-45}
HZ_SAMPLE_SEC=${HZ_SAMPLE_SEC:-6}
MIN_CAMERA_HZ=${MIN_CAMERA_HZ:-15}
MIN_IMU_HZ=${MIN_IMU_HZ:-200}
MIN_ODOM_HZ=${MIN_ODOM_HZ:-15}

RATE_LOG_DIR=$(mktemp -d)
cleanup() {
  rm -rf "${RATE_LOG_DIR}"
}
trap cleanup EXIT

required_topics=(
  "/${SCOUT_NAME}/stereo/left/image_raw"
  "/${SCOUT_NAME}/stereo/right/image_raw"
  "/${SCOUT_NAME}/stereo/left/camera_info"
  "/${SCOUT_NAME}/stereo/right/camera_info"
  "/${SCOUT_NAME}/imu/data"
  "/${SCOUT_NAME}/openvins/odom"
  "/map"
  "/rtabmap/cloud_map"
)

sample_rate_hz() {
  local topic=$1
  local log_file="${RATE_LOG_DIR}/$(echo "${topic}" | tr '/:' '__').log"
  timeout "${HZ_SAMPLE_SEC}"s ros2 topic hz "${topic}" > "${log_file}" 2>&1 || true

  local rate
  if ! rate=$(PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}" python3 - "${log_file}" <<'PY'
import sys
from pathlib import Path

from validation_utils import parse_average_rate

text = Path(sys.argv[1]).read_text()
rate = parse_average_rate(text)
if rate is None:
    raise SystemExit(1)
print(rate)
PY
  ); then
    echo "[validate_slam_topics] ERROR: unable to parse average rate for ${topic}" >&2
    echo "[validate_slam_topics] --- ros2 topic hz output (${topic}) ---" >&2
    cat "${log_file}" >&2
    return 1
  fi

  echo "${rate}"
}

assert_min_rate() {
  local topic=$1
  local measured_hz=$2
  local min_hz=$3
  if ! python3 - "${measured_hz}" "${min_hz}" <<'PY'
import sys

measured = float(sys.argv[1])
minimum = float(sys.argv[2])
if measured < minimum:
    raise SystemExit(1)
PY
  then
    echo "[validate_slam_topics] ERROR: ${topic} average ${measured_hz} Hz is below minimum ${min_hz} Hz" >&2
    return 1
  fi

  echo "[validate_slam_topics] ${topic}: ${measured_hz} Hz (min ${min_hz} Hz)"
}

echo "[validate_slam_topics] Waiting for required topics (${TIMEOUT_SEC}s timeout)..."
end=$((SECONDS + TIMEOUT_SEC))
while (( SECONDS < end )); do
  all_found=1
  for topic in "${required_topics[@]}"; do
    if ! ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      all_found=0
      break
    fi
  done

  if (( all_found == 1 )); then
    echo "[validate_slam_topics] All required topics are visible."
    break
  fi
  sleep 1
done

for topic in "${required_topics[@]}"; do
  if ! ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
    echo "[validate_slam_topics] ERROR: missing topic ${topic}" >&2
    exit 1
  fi
done

echo "[validate_slam_topics] Sampling publish rates with ros2 topic hz..."
left_rate=$(sample_rate_hz "/${SCOUT_NAME}/stereo/left/image_raw")
right_rate=$(sample_rate_hz "/${SCOUT_NAME}/stereo/right/image_raw")
imu_rate=$(sample_rate_hz "/${SCOUT_NAME}/imu/data")
odom_rate=$(sample_rate_hz "/${SCOUT_NAME}/openvins/odom")

assert_min_rate "/${SCOUT_NAME}/stereo/left/image_raw" "${left_rate}" "${MIN_CAMERA_HZ}"
assert_min_rate "/${SCOUT_NAME}/stereo/right/image_raw" "${right_rate}" "${MIN_CAMERA_HZ}"
assert_min_rate "/${SCOUT_NAME}/imu/data" "${imu_rate}" "${MIN_IMU_HZ}"
assert_min_rate "/${SCOUT_NAME}/openvins/odom" "${odom_rate}" "${MIN_ODOM_HZ}"

echo "[validate_slam_topics] OK"
