#!/usr/bin/env bash
set -euo pipefail

SCOUT_NAME=${SCOUT_NAME:-scout1}
TIMEOUT_SEC=${TIMEOUT_SEC:-45}
HZ_SAMPLE_SEC=${HZ_SAMPLE_SEC:-6}

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
for topic in "/${SCOUT_NAME}/stereo/left/image_raw" "/${SCOUT_NAME}/stereo/right/image_raw" "/${SCOUT_NAME}/imu/data"; do
  echo "[validate_slam_topics] ${topic}"
  timeout "${HZ_SAMPLE_SEC}"s ros2 topic hz "${topic}" || true
done

echo "[validate_slam_topics] OK"
