#!/usr/bin/env bash
#
# validate_thermal_detection_rate.sh - Thermal pipeline throughput validation
#
# DESCRIPTION:
#   End-to-end smoke test for thermal detection throughput in simulation.
#   Waits for thermal image input and hotspot output topics, samples output
#   publish rate via `ros2 topic hz`, and asserts the configured minimum.
#
# USAGE:
#   ./validate_thermal_detection_rate.sh
#
# ENVIRONMENT VARIABLES:
#   SCOUT_NAME       - Vehicle namespace for thermal image input (default: scout1)
#   THERMAL_TOPIC    - Thermal hotspot output topic (default: thermal/hotspots)
#   TIMEOUT_SEC      - Seconds to wait for topics to appear (default: 45)
#   HZ_SAMPLE_SEC    - Seconds for ros2 topic hz sampling (default: 8)
#   MIN_HOTSPOT_HZ   - Minimum required hotspot throughput (default: 5)
#

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)

SCOUT_NAME=${SCOUT_NAME:-scout1}
THERMAL_TOPIC=${THERMAL_TOPIC:-/thermal/hotspots}
THERMAL_IMAGE_TOPIC=${THERMAL_IMAGE_TOPIC:-"/${SCOUT_NAME}/thermal/image_raw"}
TIMEOUT_SEC=${TIMEOUT_SEC:-45}
HZ_SAMPLE_SEC=${HZ_SAMPLE_SEC:-8}
MIN_HOTSPOT_HZ=${MIN_HOTSPOT_HZ:-5}

# Normalize topics to absolute ROS names for ros2 topic list comparisons
if [[ "${THERMAL_TOPIC}" != /* ]]; then
  THERMAL_TOPIC="/${THERMAL_TOPIC}"
fi
if [[ "${THERMAL_IMAGE_TOPIC}" != /* ]]; then
  THERMAL_IMAGE_TOPIC="/${THERMAL_IMAGE_TOPIC}"
fi

RATE_LOG_DIR=$(mktemp -d)
cleanup() {
  rm -rf "${RATE_LOG_DIR}"
}
trap cleanup EXIT

required_topics=(
  "${THERMAL_IMAGE_TOPIC}"
  "${THERMAL_TOPIC}"
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
    echo "[validate_thermal_detection_rate] ERROR: unable to parse average rate for ${topic}" >&2
    echo "[validate_thermal_detection_rate] --- ros2 topic hz output (${topic}) ---" >&2
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
    echo "[validate_thermal_detection_rate] ERROR: ${topic} average ${measured_hz} Hz is below minimum ${min_hz} Hz" >&2
    return 1
  fi

  echo "[validate_thermal_detection_rate] ${topic}: ${measured_hz} Hz (min ${min_hz} Hz)"
}

echo "[validate_thermal_detection_rate] Waiting for thermal topics (${TIMEOUT_SEC}s timeout)..."
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
    echo "[validate_thermal_detection_rate] Thermal topics discovered."
    break
  fi
  sleep 1
done

for topic in "${required_topics[@]}"; do
  if ! ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
    echo "[validate_thermal_detection_rate] ERROR: missing topic ${topic}" >&2
    exit 1
  fi
done

echo "[validate_thermal_detection_rate] Sampling throughput with ros2 topic hz..."
image_rate=$(sample_rate_hz "${THERMAL_IMAGE_TOPIC}")
hotspot_rate=$(sample_rate_hz "${THERMAL_TOPIC}")

echo "[validate_thermal_detection_rate] ${THERMAL_IMAGE_TOPIC}: ${image_rate} Hz (input)"
assert_min_rate "${THERMAL_TOPIC}" "${hotspot_rate}" "${MIN_HOTSPOT_HZ}"
echo "[validate_thermal_detection_rate] OK"
