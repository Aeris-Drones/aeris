#!/usr/bin/env bash
#
# validate_multi_vehicle_dds.sh - Multi-vehicle DDS routing/QoS validation recipe
#
# DESCRIPTION:
#   Launches the multi-drone simulation with explicit DDS profile wiring,
#   validates topic discovery/routing across scout1, scout2, ranger1, and
#   runs nominal vs impaired communication checks.
#
#   The impairment phase reuses aeris_mesh_agent.impairment_relay and toggles
#   link degradation on/off using ros2 parameter updates.
#
# USAGE:
#   ./software/sim/tools/validate_multi_vehicle_dds.sh
#
# ENVIRONMENT VARIABLES:
#   CYCLONEDDS_URI                 Default: file://<repo>/software/edge/config/dds/cyclonedds.xml
#   FASTRTPS_DEFAULT_PROFILES_FILE Default: <repo>/software/edge/config/dds/fastdds.xml
#   RMW_IMPLEMENTATION             Default: rmw_cyclonedds_cpp
#   TOPIC_TIMEOUT_SEC              Default: 120
#   HZ_SAMPLE_SEC                  Default: 8
#   IMPAIRMENT_DROP_PROB           Default: 0.30
#   IMPAIRMENT_DELAY_MS            Default: 120
#   FLOOD_RATE_HZ                  Default: 200
#   FLOOD_PAYLOAD_BYTES            Default: 512
#   LOG_DIR                        Default: <repo>/output/dds_multi_vehicle

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/../../.." && pwd)

CYCLONEDDS_URI=${CYCLONEDDS_URI:-"file://${REPO_ROOT}/software/edge/config/dds/cyclonedds.xml"}
FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-"${REPO_ROOT}/software/edge/config/dds/fastdds.xml"}
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
TOPIC_TIMEOUT_SEC=${TOPIC_TIMEOUT_SEC:-120}
HZ_SAMPLE_SEC=${HZ_SAMPLE_SEC:-8}
IMPAIRMENT_DROP_PROB=${IMPAIRMENT_DROP_PROB:-0.30}
IMPAIRMENT_DELAY_MS=${IMPAIRMENT_DELAY_MS:-120}
FLOOD_RATE_HZ=${FLOOD_RATE_HZ:-200}
FLOOD_PAYLOAD_BYTES=${FLOOD_PAYLOAD_BYTES:-512}
LOG_DIR=${LOG_DIR:-"${REPO_ROOT}/output/dds_multi_vehicle"}

SIM_PID=""
RELAY_PID=""
FLOOD_PID=""

mkdir -p "${LOG_DIR}"

require_cmd() {
  local cmd=$1
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: required command not found: ${cmd}" >&2
    exit 1
  fi
}

cleanup() {
  set +e
  if [[ -n "${FLOOD_PID}" ]] && kill -0 "${FLOOD_PID}" >/dev/null 2>&1; then
    kill "${FLOOD_PID}" >/dev/null 2>&1
    wait "${FLOOD_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${RELAY_PID}" ]] && kill -0 "${RELAY_PID}" >/dev/null 2>&1; then
    kill "${RELAY_PID}" >/dev/null 2>&1
    wait "${RELAY_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" >/dev/null 2>&1; then
    kill "${SIM_PID}" >/dev/null 2>&1
    wait "${SIM_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

wait_for_topic() {
  local topic=$1
  local end=$((SECONDS + TOPIC_TIMEOUT_SEC))
  while (( SECONDS < end )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      echo "[validate_multi_vehicle_dds] Topic discovered: ${topic}"
      return 0
    fi
    sleep 1
  done
  echo "[validate_multi_vehicle_dds] ERROR: topic not discovered within timeout: ${topic}" >&2
  return 1
}

wait_for_namespace() {
  local ns=$1
  local end=$((SECONDS + TOPIC_TIMEOUT_SEC))
  while (( SECONDS < end )); do
    if ros2 topic list 2>/dev/null | grep -Eq "^/${ns}/"; then
      echo "[validate_multi_vehicle_dds] Namespace active: /${ns}/"
      return 0
    fi
    sleep 1
  done
  echo "[validate_multi_vehicle_dds] ERROR: namespace topics not discovered: /${ns}/" >&2
  return 1
}

sample_topic_hz() {
  local topic=$1
  local label=$2
  local out_file="${LOG_DIR}/topic_hz_${label}_$(echo "${topic}" | tr '/:' '__').log"
  timeout "${HZ_SAMPLE_SEC}"s ros2 topic hz "${topic}" >"${out_file}" 2>&1 || true
  echo "[validate_multi_vehicle_dds] Captured rate sample: ${topic} (${label}) -> ${out_file}"
}

export CYCLONEDDS_URI
export FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_IMPLEMENTATION

require_cmd ros2
require_cmd timeout
require_cmd grep
require_cmd tr

echo "[validate_multi_vehicle_dds] Logs: ${LOG_DIR}"
echo "[validate_multi_vehicle_dds] CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "[validate_multi_vehicle_dds] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "[validate_multi_vehicle_dds] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

ros2 launch aeris_sim multi_drone_sim.launch.py \
  cyclonedds_uri:="${CYCLONEDDS_URI}" \
  fastdds_profiles_file:="${FASTRTPS_DEFAULT_PROFILES_FILE}" \
  rmw_implementation:="${RMW_IMPLEMENTATION}" \
  launch_orchestrator:=true >"${LOG_DIR}/multi_drone_sim.launch.log" 2>&1 &
SIM_PID=$!

echo "[validate_multi_vehicle_dds] Waiting for nominal topic discovery..."
wait_for_topic "/map/tiles"
wait_for_topic "/orchestrator/heartbeat"
wait_for_namespace "scout1"
wait_for_namespace "scout2"
wait_for_namespace "ranger1"

ros2 topic info -v /map/tiles >"${LOG_DIR}/topic_info_map_tiles.log"
ros2 topic info -v /orchestrator/heartbeat >"${LOG_DIR}/topic_info_orchestrator_heartbeat.log"
sample_topic_hz "/map/tiles" "nominal"
sample_topic_hz "/orchestrator/heartbeat" "nominal"

ros2 run aeris_tools dds_flood_tester \
  --topic /flood/test \
  --rate "${FLOOD_RATE_HZ}" \
  --payload-bytes "${FLOOD_PAYLOAD_BYTES}" >"${LOG_DIR}/dds_flood_tester.log" 2>&1 &
FLOOD_PID=$!
sleep 2
sample_topic_hz "/flood/test" "nominal"
kill "${FLOOD_PID}" >/dev/null 2>&1 || true
wait "${FLOOD_PID}" >/dev/null 2>&1 || true
FLOOD_PID=""

ros2 run aeris_mesh_agent impairment_relay --ros-args \
  -p input_topic:=orchestrator/heartbeat \
  -p output_topic:=mesh/heartbeat_imp \
  -p drop_prob:=0.0 \
  -p delay_ms:=0 >"${LOG_DIR}/impairment_relay.log" 2>&1 &
RELAY_PID=$!

wait_for_topic "/mesh/heartbeat_imp"
ros2 topic info -v /mesh/heartbeat_imp >"${LOG_DIR}/topic_info_mesh_heartbeat_imp.log"
sample_topic_hz "/mesh/heartbeat_imp" "baseline"

echo "[validate_multi_vehicle_dds] Enabling impairment: drop_prob=${IMPAIRMENT_DROP_PROB}, delay_ms=${IMPAIRMENT_DELAY_MS}"
ros2 param set /impairment_relay drop_prob "${IMPAIRMENT_DROP_PROB}" >"${LOG_DIR}/param_set_drop_prob_on.log"
ros2 param set /impairment_relay delay_ms "${IMPAIRMENT_DELAY_MS}" >"${LOG_DIR}/param_set_delay_ms_on.log"
sleep 2
sample_topic_hz "/mesh/heartbeat_imp" "impaired"

echo "[validate_multi_vehicle_dds] Disabling impairment"
ros2 param set /impairment_relay drop_prob 0.0 >"${LOG_DIR}/param_set_drop_prob_off.log"
ros2 param set /impairment_relay delay_ms 0 >"${LOG_DIR}/param_set_delay_ms_off.log"
sleep 2
sample_topic_hz "/mesh/heartbeat_imp" "restored"

echo "[validate_multi_vehicle_dds] Validation recipe complete. Review logs under ${LOG_DIR}."
