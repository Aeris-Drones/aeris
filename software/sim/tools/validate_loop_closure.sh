#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SCOUT_NAME=${SCOUT_NAME:-scout1}
TRAJECTORY_JSON=${TRAJECTORY_JSON:-software/sim/config/loop_closure_path.json}
OUTPUT_DIR=${OUTPUT_DIR:-output/rtabmap_loop_closure}
INFO_SAMPLE_SEC=${INFO_SAMPLE_SEC:-45}
ODOM_SAMPLE_SEC=${ODOM_SAMPLE_SEC:-45}
ODOM_TOPIC=${ODOM_TOPIC:-"/${SCOUT_NAME}/openvins/odom"}
TRAJECTORY_CLOSURE_TOLERANCE_M=${TRAJECTORY_CLOSURE_TOLERANCE_M:-3.0}
START_END_TOLERANCE_M=${START_END_TOLERANCE_M:-3.0}
CLOSURE_TOKEN_REGEX=${CLOSURE_TOKEN_REGEX:-"loop|closure|optimized|constraint"}

mkdir -p "${OUTPUT_DIR}"
INFO_LOG="${OUTPUT_DIR}/rtabmap_info.log"
ODOM_LOG="${OUTPUT_DIR}/openvins_odom.log"
TF_MAP_ODOM_LOG="${OUTPUT_DIR}/tf_map_odom.log"
TF_ODOM_BASE_LOG="${OUTPUT_DIR}/tf_odom_base_link.log"

echo "[validate_loop_closure] Using trajectory: ${TRAJECTORY_JSON}"
if [[ ! -f "${TRAJECTORY_JSON}" ]]; then
  echo "[validate_loop_closure] ERROR: trajectory file missing: ${TRAJECTORY_JSON}" >&2
  exit 1
fi

if ! expected_endpoint=$(PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}" python3 - "${TRAJECTORY_JSON}" "${TRAJECTORY_CLOSURE_TOLERANCE_M}" <<'PY'
import sys
from pathlib import Path

from validation_utils import is_closed_loop, load_trajectory_waypoints

trajectory_path = Path(sys.argv[1])
tolerance_m = float(sys.argv[2])
waypoints = load_trajectory_waypoints(trajectory_path)
if not is_closed_loop(waypoints, tolerance_m):
    raise SystemExit(
        f"trajectory does not return to start within {tolerance_m:.2f}m "
        f"(start={waypoints[0]}, end={waypoints[-1]})"
    )
endpoint = waypoints[-1]
print(f"{endpoint.x} {endpoint.y} {endpoint.z}")
PY
); then
  echo "[validate_loop_closure] ERROR: invalid loop trajectory in ${TRAJECTORY_JSON}" >&2
  exit 1
fi

read -r expected_x expected_y expected_z <<< "${expected_endpoint}"

echo "[validate_loop_closure] Capture /rtabmap/info for ${INFO_SAMPLE_SEC}s"
timeout "${INFO_SAMPLE_SEC}"s ros2 topic echo /rtabmap/info > "${INFO_LOG}" 2>&1 || true &
info_pid=$!

echo "[validate_loop_closure] Capture ${ODOM_TOPIC} for ${ODOM_SAMPLE_SEC}s"
timeout "${ODOM_SAMPLE_SEC}"s ros2 topic echo "${ODOM_TOPIC}" > "${ODOM_LOG}" 2>&1 || true &
odom_pid=$!

wait "${info_pid}" || true
wait "${odom_pid}" || true

if ! PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}" python3 - "${INFO_LOG}" "${CLOSURE_TOKEN_REGEX}" <<'PY'
import sys
from pathlib import Path

from validation_utils import has_loop_closure_signal

info_log = Path(sys.argv[1]).read_text()
pattern = sys.argv[2]
if not has_loop_closure_signal(info_log, pattern):
    raise SystemExit(1)
PY
then
  echo "[validate_loop_closure] ERROR: no loop-closure signal found in ${INFO_LOG}" >&2
  exit 1
fi

if ! PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}" python3 - "${ODOM_LOG}" "${expected_x}" "${expected_y}" "${expected_z}" "${START_END_TOLERANCE_M}" <<'PY'
import sys
from pathlib import Path

from validation_utils import Point3D, distance, extract_odom_positions, is_pose_near

odom_log = Path(sys.argv[1]).read_text()
target = Point3D(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
tolerance_m = float(sys.argv[5])
samples = extract_odom_positions(odom_log)
if len(samples) < 2:
    raise SystemExit(1)
final_pose = samples[-1]
if not is_pose_near(final_pose, target, tolerance_m):
    delta = distance(final_pose, target)
    raise SystemExit(
        f"final odom pose {final_pose} too far from loop endpoint {target} (delta={delta:.3f}m, tolerance={tolerance_m:.3f}m)"
    )
print(
    f"final odom pose {final_pose} is within {tolerance_m:.2f}m of endpoint {target}"
)
PY
then
  echo "[validate_loop_closure] ERROR: odometry did not return near expected loop endpoint" >&2
  exit 1
fi

echo "[validate_loop_closure] Capture TF chain map->odom and odom->base_link"
timeout 15s ros2 run tf2_ros tf2_echo map odom > "${TF_MAP_ODOM_LOG}" 2>&1 || true
timeout 15s ros2 run tf2_ros tf2_echo odom base_link > "${TF_ODOM_BASE_LOG}" 2>&1 || true

if ! grep -q "Translation" "${TF_MAP_ODOM_LOG}"; then
  echo "[validate_loop_closure] ERROR: missing TF evidence for map->odom in ${TF_MAP_ODOM_LOG}" >&2
  exit 1
fi
if ! grep -q "Translation" "${TF_ODOM_BASE_LOG}"; then
  echo "[validate_loop_closure] ERROR: missing TF evidence for odom->base_link in ${TF_ODOM_BASE_LOG}" >&2
  exit 1
fi

echo "[validate_loop_closure] Loop-closure signal detected in ${INFO_LOG}"
echo "[validate_loop_closure] TF chain evidence captured in ${TF_MAP_ODOM_LOG} and ${TF_ODOM_BASE_LOG}"
echo "[validate_loop_closure] Artifacts saved under ${OUTPUT_DIR}"
