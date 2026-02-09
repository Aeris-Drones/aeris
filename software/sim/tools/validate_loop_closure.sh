#!/usr/bin/env bash
set -euo pipefail

SCOUT_NAME=${SCOUT_NAME:-scout1}
TRAJECTORY_JSON=${TRAJECTORY_JSON:-software/sim/config/loop_closure_path.json}
OUTPUT_DIR=${OUTPUT_DIR:-output/rtabmap_loop_closure}
INFO_SAMPLE_SEC=${INFO_SAMPLE_SEC:-45}

mkdir -p "${OUTPUT_DIR}"
INFO_LOG="${OUTPUT_DIR}/rtabmap_info.log"
TF_LOG="${OUTPUT_DIR}/tf_map_odom_base_link.log"

echo "[validate_loop_closure] Using trajectory: ${TRAJECTORY_JSON}"
if [[ ! -f "${TRAJECTORY_JSON}" ]]; then
  echo "[validate_loop_closure] ERROR: trajectory file missing: ${TRAJECTORY_JSON}" >&2
  exit 1
fi

echo "[validate_loop_closure] Capture /rtabmap/info for ${INFO_SAMPLE_SEC}s"
timeout "${INFO_SAMPLE_SEC}"s ros2 topic echo /rtabmap/info > "${INFO_LOG}" || true

echo "[validate_loop_closure] Capture TF chain map->odom->base_link"
timeout 15s ros2 run tf2_ros tf2_echo map base_link > "${TF_LOG}" || true

if grep -Eiq "loop|closure|optimized|constraint" "${INFO_LOG}"; then
  echo "[validate_loop_closure] Loop-closure signal detected in ${INFO_LOG}"
else
  echo "[validate_loop_closure] WARNING: no explicit loop-closure token found in ${INFO_LOG}" >&2
  echo "[validate_loop_closure] Review rtabmap logs and rerun with longer INFO_SAMPLE_SEC if needed." >&2
fi

echo "[validate_loop_closure] Artifacts saved under ${OUTPUT_DIR}"
