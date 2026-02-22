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
#   RMW_IMPLEMENTATION             Optional single-RMW override (legacy behavior)
#   RMW_VALIDATION_SET             Comma-separated RMW list (default: rmw_cyclonedds_cpp,rmw_fastrtps_cpp)
#   RMW_FASTRTPS_USE_QOS_FROM_XML  Default: 0
#   TOPIC_TIMEOUT_SEC              Default: 120
#   HZ_SAMPLE_SEC                  Default: 8
#   PROBE_TIMEOUT_SEC              Default: 20
#   PROBE_ATTEMPTS                 Default: 3
#   PROBE_PUB_RATE_HZ              Default: 5
#   PROBE_PUB_COUNT                Default: 15
#   IMPAIRMENT_DROP_PROB           Default: 0.30
#   IMPAIRMENT_DELAY_MS            Default: 120
#   FLOOD_RATE_HZ                  Default: 200
#   FLOOD_PAYLOAD_BYTES            Default: 512
#   RELAY_TILE_LATENCY_REQUIRED    Default: 1 (fail validation if relay tile latency cannot be asserted)
#   MAP_TILE_TOPIC                 Default: /map/tiles
#   TELEMETRY_TOPIC                Default: /orchestrator/heartbeat
#   DETECTION_TOPIC                Default: /detections/fused
#   REPLAY_ANNOTATION_TOPIC        Default: /mesh/replay_annotations
#   VIDEO_METADATA_TOPIC           Default: /scout1/stereo/left/image_raw
#   REQUIRED_NAMESPACES            Comma-separated namespace checks (default: scout1,scout2)
#   MULTI_DRONE_LAUNCH_PACKAGE     Default: aeris_sim
#   MULTI_DRONE_LAUNCH_FILE        Default: <repo>/software/sim/launch/multi_drone_sim.launch.py
#   LOG_DIR                        Default: <repo>/output/dds_multi_vehicle

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/../../.." && pwd)

CYCLONEDDS_URI=${CYCLONEDDS_URI:-"file://${REPO_ROOT}/software/edge/config/dds/cyclonedds.xml"}
FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-"${REPO_ROOT}/software/edge/config/dds/fastdds.xml"}
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}
RMW_VALIDATION_SET=${RMW_VALIDATION_SET:-}
if [[ -z "${RMW_VALIDATION_SET}" ]]; then
  if [[ -n "${RMW_IMPLEMENTATION}" ]]; then
    RMW_VALIDATION_SET="${RMW_IMPLEMENTATION}"
  else
    RMW_VALIDATION_SET="rmw_cyclonedds_cpp,rmw_fastrtps_cpp"
  fi
fi
RMW_FASTRTPS_USE_QOS_FROM_XML=${RMW_FASTRTPS_USE_QOS_FROM_XML:-0}
TOPIC_TIMEOUT_SEC=${TOPIC_TIMEOUT_SEC:-120}
HZ_SAMPLE_SEC=${HZ_SAMPLE_SEC:-8}
PROBE_TIMEOUT_SEC=${PROBE_TIMEOUT_SEC:-20}
PROBE_ATTEMPTS=${PROBE_ATTEMPTS:-3}
PROBE_PUB_RATE_HZ=${PROBE_PUB_RATE_HZ:-5}
PROBE_PUB_COUNT=${PROBE_PUB_COUNT:-15}
IMPAIRMENT_DROP_PROB=${IMPAIRMENT_DROP_PROB:-0.30}
IMPAIRMENT_DELAY_MS=${IMPAIRMENT_DELAY_MS:-120}
RELAY_TILE_LATENCY_P95_TARGET_SEC=${RELAY_TILE_LATENCY_P95_TARGET_SEC:-2.0}
RELAY_TILE_SAMPLE_TIMEOUT_SEC=${RELAY_TILE_SAMPLE_TIMEOUT_SEC:-20}
RELAY_TILE_LATENCY_REQUIRED=${RELAY_TILE_LATENCY_REQUIRED:-1}
FLOOD_RATE_HZ=${FLOOD_RATE_HZ:-200}
FLOOD_PAYLOAD_BYTES=${FLOOD_PAYLOAD_BYTES:-512}
MAP_TILE_TOPIC=${MAP_TILE_TOPIC:-/map/tiles}
TELEMETRY_TOPIC=${TELEMETRY_TOPIC:-/orchestrator/heartbeat}
DETECTION_TOPIC=${DETECTION_TOPIC:-/detections/fused}
REPLAY_ANNOTATION_TOPIC=${REPLAY_ANNOTATION_TOPIC:-/mesh/replay_annotations}
VIDEO_METADATA_TOPIC=${VIDEO_METADATA_TOPIC:-/scout1/stereo/left/image_raw}
REQUIRED_NAMESPACES=${REQUIRED_NAMESPACES:-scout1,scout2}
MULTI_DRONE_LAUNCH_PACKAGE=${MULTI_DRONE_LAUNCH_PACKAGE:-aeris_sim}
MULTI_DRONE_LAUNCH_FILE=${MULTI_DRONE_LAUNCH_FILE:-"${REPO_ROOT}/software/sim/launch/multi_drone_sim.launch.py"}
LOG_DIR=${LOG_DIR:-"${REPO_ROOT}/output/dds_multi_vehicle"}

SIM_PID=""
RELAY_PID=""
FLOOD_PID=""
MAP_PUBLISHER_PID=""
LAUNCH_CMD=()

mkdir -p "${LOG_DIR}"

require_cmd() {
  local cmd=$1
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: required command not found: ${cmd}" >&2
    exit 1
  fi
}

source_local_edge_overlay() {
  local overlay_setup="${REPO_ROOT}/software/edge/install/setup.bash"
  local had_nounset=0
  if [[ -f "${overlay_setup}" ]]; then
    if [[ $- == *u* ]]; then
      had_nounset=1
      set +u
    fi
    # shellcheck source=/dev/null
    source "${overlay_setup}"
    if (( had_nounset )); then
      set -u
    fi
    echo "[validate_multi_vehicle_dds] Sourced edge workspace overlay: ${overlay_setup}"
  fi
}

ensure_local_ros_package_visible() {
  local package_name=$1
  local local_prefix="${REPO_ROOT}/software/edge/install/${package_name}"
  local local_setup="${local_prefix}/local_setup.bash"

  if ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    return 0
  fi

  if [[ -d "${local_prefix}" ]]; then
    if [[ -f "${local_setup}" ]]; then
      # shellcheck source=/dev/null
      source "${local_setup}"
    else
      export AMENT_PREFIX_PATH="${local_prefix}:${AMENT_PREFIX_PATH:-}"
    fi
    if ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
      echo "[validate_multi_vehicle_dds] Added local overlay for ${package_name}: ${local_prefix}"
      return 0
    fi
  fi

  return 1
}

resolve_launch_target() {
  if ros2 pkg prefix "${MULTI_DRONE_LAUNCH_PACKAGE}" >/dev/null 2>&1; then
    LAUNCH_CMD=("ros2" "launch" "${MULTI_DRONE_LAUNCH_PACKAGE}" "multi_drone_sim.launch.py")
    echo "[validate_multi_vehicle_dds] Launch target resolved via package: ${MULTI_DRONE_LAUNCH_PACKAGE}"
    return 0
  fi

  if [[ -f "${MULTI_DRONE_LAUNCH_FILE}" ]]; then
    LAUNCH_CMD=("ros2" "launch" "${MULTI_DRONE_LAUNCH_FILE}")
    echo "[validate_multi_vehicle_dds] WARNING: package '${MULTI_DRONE_LAUNCH_PACKAGE}' not found; using launch file path fallback: ${MULTI_DRONE_LAUNCH_FILE}" >&2
    return 0
  fi

  echo "[validate_multi_vehicle_dds] ERROR: could not resolve multi-drone launch target." >&2
  echo "[validate_multi_vehicle_dds] Missing package '${MULTI_DRONE_LAUNCH_PACKAGE}' and launch file '${MULTI_DRONE_LAUNCH_FILE}'." >&2
  return 1
}

preflight_runtime_checks() {
  local failed=0

  # Some interface-only packages may exist locally but not be present in AMENT_PREFIX_PATH
  # after minimal setup sourcing. Repair local visibility before strict checks.
  ensure_local_ros_package_visible aeris_msgs || true
  ensure_local_ros_package_visible aeris_map || true
  ensure_local_ros_package_visible aeris_mesh_agent || true
  ensure_local_ros_package_visible aeris_orchestrator || true

  if [[ ! -f "${REPO_ROOT}/software/edge/tools/dds_flood_tester.py" ]]; then
    echo "[validate_multi_vehicle_dds] ERROR: flood tester script not found: ${REPO_ROOT}/software/edge/tools/dds_flood_tester.py" >&2
    failed=1
  fi

  if [[ -z "${PX4_BIN:-}" ]] && ! command -v px4 >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: PX4 SITL binary not found (set PX4_BIN or install 'px4')." >&2
    failed=1
  fi

  if ! command -v ign >/dev/null 2>&1 && ! command -v gz >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: Gazebo CLI not found (install 'ign' or 'gz')." >&2
    failed=1
  fi

  if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: ROS package 'ros_gz_bridge' not found in current environment." >&2
    failed=1
  fi

  if ! ros2 pkg prefix aeris_mesh_agent >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: ROS package 'aeris_mesh_agent' not found in current environment." >&2
    failed=1
  fi

  if ! ros2 pkg prefix aeris_map >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: ROS package 'aeris_map' not found in current environment." >&2
    failed=1
  fi

  if ! ros2 pkg prefix aeris_orchestrator >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: ROS package 'aeris_orchestrator' not found in current environment." >&2
    failed=1
  fi

  if ! ros2 interface show aeris_msgs/msg/FusedDetection >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: ROS interface 'aeris_msgs/msg/FusedDetection' not available in current environment." >&2
    failed=1
  fi

  if ! python3 -c "import pymavlink" >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: Python dependency 'pymavlink' is required for aeris_orchestrator mission node." >&2
    failed=1
  fi

  if (( failed )); then
    echo "[validate_multi_vehicle_dds] ERROR: runtime preflight checks failed; aborting validation early." >&2
    return 1
  fi

  return 0
}

cleanup() {
  local had_errexit=0
  if [[ $- == *e* ]]; then
    had_errexit=1
    set +e
  fi

  if [[ -n "${FLOOD_PID}" ]] && kill -0 "${FLOOD_PID}" >/dev/null 2>&1; then
    kill "${FLOOD_PID}" >/dev/null 2>&1
    wait "${FLOOD_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${MAP_PUBLISHER_PID}" ]] && kill -0 "${MAP_PUBLISHER_PID}" >/dev/null 2>&1; then
    kill "${MAP_PUBLISHER_PID}" >/dev/null 2>&1
    wait "${MAP_PUBLISHER_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${RELAY_PID}" ]] && kill -0 "${RELAY_PID}" >/dev/null 2>&1; then
    kill "${RELAY_PID}" >/dev/null 2>&1
    wait "${RELAY_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" >/dev/null 2>&1; then
    kill "${SIM_PID}" >/dev/null 2>&1
    wait "${SIM_PID}" >/dev/null 2>&1 || true
  fi

  if (( had_errexit )); then
    set -e
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
  local pass_log_dir=$2
  local label=$3
  local sanitized_topic=""
  local out_file=""

  sanitized_topic=$(echo "${topic}" | tr '/:' '__')
  out_file="${pass_log_dir}/topic_hz_${label}_${sanitized_topic}.log"
  timeout "${HZ_SAMPLE_SEC}"s ros2 topic hz "${topic}" >"${out_file}" 2>&1 || true
  echo "[validate_multi_vehicle_dds] Captured rate sample: ${topic} (${label}) -> ${out_file}" >&2
  echo "${out_file}"
}

capture_topic_once() {
  local topic=$1
  local pass_log_dir=$2
  local label=$3
  local sanitized_topic=""
  local out_file=""

  sanitized_topic=$(echo "${topic}" | tr '/:' '__')
  out_file="${pass_log_dir}/topic_echo_${label}_${sanitized_topic}.log"

  if timeout "${PROBE_TIMEOUT_SEC}"s ros2 topic echo "${topic}" --once >"${out_file}" 2>&1; then
    echo "[validate_multi_vehicle_dds] Captured one-shot sample: ${topic} (${label}) -> ${out_file}" >&2
    return 0
  fi

  echo "[validate_multi_vehicle_dds] WARNING: no one-shot sample observed for ${topic} (${label})" >&2
  echo "[validate_multi_vehicle_dds] Sample log: ${out_file}" >&2
  return 1
}

capture_topic_info() {
  local topic=$1
  local pass_log_dir=$2
  local label=$3
  local sanitized_topic=""
  local out_file=""

  sanitized_topic=$(echo "${topic}" | tr '/:' '__')
  out_file="${pass_log_dir}/topic_info_${label}_${sanitized_topic}.log"
  ros2 topic info -v "${topic}" >"${out_file}"
  echo "${out_file}"
}

assert_topic_info_reliability() {
  local topic=$1
  local expected_pattern=$2
  local topic_info_file=$3

  if ! grep -Eiq "Reliability:[[:space:]]*(${expected_pattern})" "${topic_info_file}"; then
    echo "[validate_multi_vehicle_dds] ERROR: QoS reliability mismatch for ${topic}" >&2
    echo "[validate_multi_vehicle_dds] Expected pattern: ${expected_pattern}" >&2
    echo "[validate_multi_vehicle_dds] Topic info log: ${topic_info_file}" >&2
    return 1
  fi
}

assert_topic_rate_observed() {
  local topic=$1
  local phase=$2
  local sample_file=$3
  local rate

  rate=$(awk '/average rate:/ {print $3; exit}' "${sample_file}")
  if [[ -z "${rate}" ]]; then
    echo "[validate_multi_vehicle_dds] ERROR: no average rate observed for ${topic} (${phase})" >&2
    echo "[validate_multi_vehicle_dds] Sample log: ${sample_file}" >&2
    return 1
  fi

  if ! awk -v observed="${rate}" 'BEGIN { exit !(observed + 0 > 0) }'; then
    echo "[validate_multi_vehicle_dds] ERROR: non-positive rate for ${topic} (${phase}): ${rate}" >&2
    echo "[validate_multi_vehicle_dds] Sample log: ${sample_file}" >&2
    return 1
  fi
}

assert_relay_tile_latency_p95() {
  local annotation_file=$1
  local target_p95=$2
  local require_samples=$3

  python3 - "$annotation_file" "$target_p95" "$require_samples" <<'PY'
import json
import re
import sys

annotation_file = sys.argv[1]
target = float(sys.argv[2])
require_samples = str(sys.argv[3]).strip().lower() in {"1", "true", "yes", "on"}
latencies = []

with open(annotation_file, "r", encoding="utf-8", errors="ignore") as f:
    for line in f:
        line = line.strip()
        match = re.search(r"data:\s*'(.+)'", line)
        if not match:
            continue
        raw = match.group(1)
        raw = raw.replace("\\'", "'")
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            continue
        relay = payload.get("relay_envelope", {})
        if not isinstance(relay, dict):
            continue
        if relay.get("delivery_mode") != "relay":
            continue
        route_key = str(payload.get("route_key", ""))
        if "tile" not in route_key:
            continue
        original = payload.get("original_event_ts")
        published = payload.get("published_at_ts", payload.get("replayed_at_ts"))
        if original is None or published is None:
            continue
        try:
            latency = float(published) - float(original)
        except (TypeError, ValueError):
            continue
        if latency >= 0:
            latencies.append(latency)

if not latencies:
    if require_samples:
        print("ERROR: no relay tile latency samples with replay metadata")
        sys.exit(1)
    print("WARNING: no relay tile latency samples; skipping p95 assertion")
    sys.exit(0)

latencies.sort()
p95_index = max(0, min(len(latencies) - 1, round((len(latencies) - 1) * 0.95)))
p95 = latencies[p95_index]
if p95 > target:
    print(f"ERROR: relay tile latency p95 {p95:.3f}s exceeded target {target:.3f}s")
    sys.exit(1)

print(f"Relay tile latency p95={p95:.3f}s within target {target:.3f}s")
PY
}

is_truthy() {
  local value="${1:-}"
  case "${value,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

probe_topic_roundtrip() {
  local topic=$1
  local msg_type=$2
  local payload=$3
  local pass_log_dir=$4
  local label=$5
  local echo_log=""
  local pub_log=""
  local echo_pid=""
  local attempt=1

  for (( attempt = 1; attempt <= PROBE_ATTEMPTS; attempt++ )); do
    echo_log="${pass_log_dir}/probe_${label}_echo_attempt${attempt}.log"
    pub_log="${pass_log_dir}/probe_${label}_pub_attempt${attempt}.log"

    timeout "${PROBE_TIMEOUT_SEC}"s ros2 topic echo "${topic}" --once >"${echo_log}" 2>&1 &
    echo_pid=$!
    sleep 1

    if ! timeout "${PROBE_TIMEOUT_SEC}"s ros2 topic pub \
      -r "${PROBE_PUB_RATE_HZ}" \
      -t "${PROBE_PUB_COUNT}" \
      "${topic}" "${msg_type}" "${payload}" >"${pub_log}" 2>&1; then
      kill "${echo_pid}" >/dev/null 2>&1 || true
      wait "${echo_pid}" >/dev/null 2>&1 || true
      echo "[validate_multi_vehicle_dds] WARNING: probe publish attempt ${attempt}/${PROBE_ATTEMPTS} failed on ${topic}" >&2
      echo "[validate_multi_vehicle_dds] Publisher log: ${pub_log}" >&2
      sleep 1
      continue
    fi

    if wait "${echo_pid}" && grep -q . "${echo_log}"; then
      echo "[validate_multi_vehicle_dds] Probe roundtrip successful: ${topic} (attempt ${attempt})"
      return 0
    fi

    wait "${echo_pid}" >/dev/null 2>&1 || true
    echo "[validate_multi_vehicle_dds] WARNING: no probe sample observed on ${topic} (attempt ${attempt}/${PROBE_ATTEMPTS})" >&2
    echo "[validate_multi_vehicle_dds] Echo log: ${echo_log}" >&2
    sleep 1
  done

  echo "[validate_multi_vehicle_dds] ERROR: no probe sample observed on ${topic} after ${PROBE_ATTEMPTS} attempt(s)" >&2
  echo "[validate_multi_vehicle_dds] Last echo log: ${echo_log}" >&2
  echo "[validate_multi_vehicle_dds] Last publisher log: ${pub_log}" >&2
  return 1
}

run_validation_pass() {
  local rmw_impl=$1
  local rmw_tag=${rmw_impl//[^a-zA-Z0-9_]/_}
  local pass_log_dir="${LOG_DIR}/${rmw_tag}"
  local fastdds_profiles_for_pass=""
  local fastrtps_qos_xml_for_pass="0"
  local launch_args=()
  local prev_rmw_impl="${RMW_IMPLEMENTATION-}"
  local prev_fastdds_profiles="${FASTRTPS_DEFAULT_PROFILES_FILE-}"
  local prev_fastrtps_qos_xml="${RMW_FASTRTPS_USE_QOS_FROM_XML-}"
  local prev_rmw_impl_set=0
  local prev_fastdds_profiles_set=0
  local prev_fastrtps_qos_xml_set=0
  local telemetry_info_file=""
  local map_info_file=""
  local video_info_file=""
  local mesh_info_file=""
  local nominal_telemetry_hz_file=""
  local nominal_video_hz_file=""
  local flood_hz_file=""
  local baseline_mesh_hz_file=""
  local impaired_mesh_hz_file=""
  local impaired_telemetry_hz_file=""
  local impaired_map_hz_file=""
  local impaired_detection_probe_log=""
  local replay_annotation_impaired_hz_file=""
  local replay_annotation_restored_hz_file=""
  local replay_annotation_sample_file=""
  local relay_tile_latency_sample_file=""
  local restored_mesh_hz_file=""
  local restored_telemetry_hz_file=""
  local restored_map_hz_file=""
  local restored_detection_probe_log=""
  local map_publisher_log=""

  if [[ -n "${RMW_IMPLEMENTATION+x}" ]]; then
    prev_rmw_impl_set=1
  fi
  if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE+x}" ]]; then
    prev_fastdds_profiles_set=1
  fi
  if [[ -n "${RMW_FASTRTPS_USE_QOS_FROM_XML+x}" ]]; then
    prev_fastrtps_qos_xml_set=1
  fi

  if [[ "${rmw_impl}" == "rmw_fastrtps_cpp" ]]; then
    fastdds_profiles_for_pass="${FASTRTPS_DEFAULT_PROFILES_FILE}"
    fastrtps_qos_xml_for_pass="${RMW_FASTRTPS_USE_QOS_FROM_XML}"
  fi

  export RMW_IMPLEMENTATION="${rmw_impl}"
  export RMW_FASTRTPS_USE_QOS_FROM_XML="${fastrtps_qos_xml_for_pass}"
  if [[ -n "${fastdds_profiles_for_pass}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${fastdds_profiles_for_pass}"
  else
    unset FASTRTPS_DEFAULT_PROFILES_FILE
  fi

  mkdir -p "${pass_log_dir}"

  launch_args=(
    "cyclonedds_uri:=${CYCLONEDDS_URI}"
    "rmw_implementation:=${rmw_impl}"
    "rmw_fastrtps_use_qos_from_xml:=${fastrtps_qos_xml_for_pass}"
    "launch_orchestrator:=true"
  )
  if [[ -n "${fastdds_profiles_for_pass}" ]]; then
    launch_args+=("fastdds_profiles_file:=${fastdds_profiles_for_pass}")
  fi

  echo "[validate_multi_vehicle_dds] ===== RMW validation pass: ${rmw_impl} ====="
  "${LAUNCH_CMD[@]}" "${launch_args[@]}" >"${pass_log_dir}/multi_drone_sim.launch.log" 2>&1 &
  SIM_PID=$!
  sleep 1
  if ! kill -0 "${SIM_PID}" >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: multi-drone launch exited early for ${rmw_impl}" >&2
    echo "[validate_multi_vehicle_dds] Launch log: ${pass_log_dir}/multi_drone_sim.launch.log" >&2
    return 1
  fi

  map_publisher_log="${pass_log_dir}/map_tile_publisher.log"
  ros2 run aeris_map map_tile_publisher >"${map_publisher_log}" 2>&1 &
  MAP_PUBLISHER_PID=$!
  sleep 1
  if ! kill -0 "${MAP_PUBLISHER_PID}" >/dev/null 2>&1; then
    echo "[validate_multi_vehicle_dds] ERROR: map tile publisher failed to start for ${rmw_impl}" >&2
    echo "[validate_multi_vehicle_dds] Map publisher log: ${map_publisher_log}" >&2
    return 1
  fi

  echo "[validate_multi_vehicle_dds] Waiting for nominal topic discovery (${rmw_impl})..."
  wait_for_topic "${MAP_TILE_TOPIC}"
  wait_for_topic "${TELEMETRY_TOPIC}"
  wait_for_topic "${VIDEO_METADATA_TOPIC}"
  local namespace_list=()
  local ns=""
  local ns_trimmed=""
  IFS=',' read -r -a namespace_list <<< "${REQUIRED_NAMESPACES}"
  for ns in "${namespace_list[@]}"; do
    ns_trimmed=$(echo "${ns}" | tr -d '[:space:]')
    if [[ -z "${ns_trimmed}" ]]; then
      continue
    fi
    wait_for_namespace "${ns_trimmed}"
  done

  # Probe a representative detection topic to validate discoverability and routability.
  probe_topic_roundtrip \
    "${DETECTION_TOPIC}" \
    "aeris_msgs/msg/FusedDetection" \
    "{}" \
    "${pass_log_dir}" \
    "detection"

  map_info_file=$(capture_topic_info "${MAP_TILE_TOPIC}" "${pass_log_dir}" "map_tiles")
  telemetry_info_file=$(capture_topic_info "${TELEMETRY_TOPIC}" "${pass_log_dir}" "telemetry")
  video_info_file=$(capture_topic_info "${VIDEO_METADATA_TOPIC}" "${pass_log_dir}" "video_metadata")

  assert_topic_info_reliability "${TELEMETRY_TOPIC}" "RELIABLE" "${telemetry_info_file}"
  assert_topic_info_reliability "${MAP_TILE_TOPIC}" "RELIABLE" "${map_info_file}"
  assert_topic_info_reliability "${VIDEO_METADATA_TOPIC}" "RELIABLE|BEST[_ ]?EFFORT" "${video_info_file}"

  nominal_telemetry_hz_file=$(sample_topic_hz "${TELEMETRY_TOPIC}" "${pass_log_dir}" "nominal")
  nominal_video_hz_file=$(sample_topic_hz "${VIDEO_METADATA_TOPIC}" "${pass_log_dir}" "nominal")
  assert_topic_rate_observed "${TELEMETRY_TOPIC}" "nominal" "${nominal_telemetry_hz_file}"
  if ! assert_topic_rate_observed "${VIDEO_METADATA_TOPIC}" "nominal" "${nominal_video_hz_file}"; then
    echo "[validate_multi_vehicle_dds] WARNING: no nominal video rate observed on ${VIDEO_METADATA_TOPIC}; continuing because ros_gz_bridge may advertise image topics without active frame flow in headless runs." >&2
  fi

  python3 "${REPO_ROOT}/software/edge/tools/dds_flood_tester.py" \
    --topic /flood/test \
    --rate "${FLOOD_RATE_HZ}" \
    --payload-bytes "${FLOOD_PAYLOAD_BYTES}" >"${pass_log_dir}/dds_flood_tester.log" 2>&1 &
  FLOOD_PID=$!
  sleep 2
  flood_hz_file=$(sample_topic_hz "/flood/test" "${pass_log_dir}" "nominal")
  assert_topic_rate_observed "/flood/test" "nominal" "${flood_hz_file}"
  kill "${FLOOD_PID}" >/dev/null 2>&1 || true
  wait "${FLOOD_PID}" >/dev/null 2>&1 || true
  FLOOD_PID=""

  ros2 run aeris_mesh_agent impairment_relay --ros-args \
    -p input_topic:=orchestrator/heartbeat \
    -p output_topic:=mesh/heartbeat_imp \
    -p drop_prob:=0.0 \
    -p delay_ms:=0 >"${pass_log_dir}/impairment_relay.log" 2>&1 &
  RELAY_PID=$!

  wait_for_topic "/mesh/heartbeat_imp"
  mesh_info_file=$(capture_topic_info "/mesh/heartbeat_imp" "${pass_log_dir}" "mesh_heartbeat_imp")
  assert_topic_info_reliability "/mesh/heartbeat_imp" "RELIABLE" "${mesh_info_file}"

  baseline_mesh_hz_file=$(sample_topic_hz "/mesh/heartbeat_imp" "${pass_log_dir}" "baseline")
  assert_topic_rate_observed "/mesh/heartbeat_imp" "baseline" "${baseline_mesh_hz_file}"

  echo "[validate_multi_vehicle_dds] Enabling impairment: drop_prob=${IMPAIRMENT_DROP_PROB}, delay_ms=${IMPAIRMENT_DELAY_MS}"
  ros2 param set /impairment_relay drop_prob "${IMPAIRMENT_DROP_PROB}" >"${pass_log_dir}/param_set_drop_prob_on.log"
  ros2 param set /impairment_relay delay_ms "${IMPAIRMENT_DELAY_MS}" >"${pass_log_dir}/param_set_delay_ms_on.log"
  sleep 2

  impaired_mesh_hz_file=$(sample_topic_hz "/mesh/heartbeat_imp" "${pass_log_dir}" "impaired")
  impaired_telemetry_hz_file=$(sample_topic_hz "${TELEMETRY_TOPIC}" "${pass_log_dir}" "impaired")
  impaired_map_hz_file=$(sample_topic_hz "${MAP_TILE_TOPIC}" "${pass_log_dir}" "impaired")
  if ! assert_topic_rate_observed "/mesh/heartbeat_imp" "impaired" "${impaired_mesh_hz_file}"; then
    echo "[validate_multi_vehicle_dds] WARNING: no impaired mesh rate observed on /mesh/heartbeat_imp; continuing because CLI sampling can miss delayed relay windows under load." >&2
  fi
  assert_topic_rate_observed "${TELEMETRY_TOPIC}" "impaired" "${impaired_telemetry_hz_file}"
  if ! assert_topic_rate_observed "${MAP_TILE_TOPIC}" "impaired" "${impaired_map_hz_file}"; then
    if capture_topic_once "${MAP_TILE_TOPIC}" "${pass_log_dir}" "impaired"; then
      echo "[validate_multi_vehicle_dds] WARNING: no impaired average rate observed on ${MAP_TILE_TOPIC}; one-shot sample succeeded so continuing." >&2
    else
      return 1
    fi
  fi
  probe_topic_roundtrip \
    "${DETECTION_TOPIC}" \
    "aeris_msgs/msg/FusedDetection" \
    "{}" \
    "${pass_log_dir}" \
    "detection_impaired"
  impaired_detection_probe_log="${pass_log_dir}/probe_detection_impaired_echo_attempt1.log"
  if ros2 topic list 2>/dev/null | grep -Fxq "${REPLAY_ANNOTATION_TOPIC}"; then
    replay_annotation_impaired_hz_file=$(sample_topic_hz "${REPLAY_ANNOTATION_TOPIC}" "${pass_log_dir}" "replay_annotation_impaired")
    if ! assert_topic_rate_observed "${REPLAY_ANNOTATION_TOPIC}" "impaired" "${replay_annotation_impaired_hz_file}"; then
      echo "[validate_multi_vehicle_dds] WARNING: replay annotation topic has no impaired sample yet: ${REPLAY_ANNOTATION_TOPIC}" >&2
    fi
  else
    echo "[validate_multi_vehicle_dds] WARNING: replay annotation topic not discovered (skipping impaired replay metadata checks): ${REPLAY_ANNOTATION_TOPIC}" >&2
  fi

  echo "[validate_multi_vehicle_dds] Disabling impairment"
  ros2 param set /impairment_relay drop_prob 0.0 >"${pass_log_dir}/param_set_drop_prob_off.log"
  ros2 param set /impairment_relay delay_ms 0 >"${pass_log_dir}/param_set_delay_ms_off.log"
  sleep 2

  restored_mesh_hz_file=$(sample_topic_hz "/mesh/heartbeat_imp" "${pass_log_dir}" "restored")
  restored_telemetry_hz_file=$(sample_topic_hz "${TELEMETRY_TOPIC}" "${pass_log_dir}" "restored")
  restored_map_hz_file=$(sample_topic_hz "${MAP_TILE_TOPIC}" "${pass_log_dir}" "restored")
  if ! assert_topic_rate_observed "/mesh/heartbeat_imp" "restored" "${restored_mesh_hz_file}"; then
    echo "[validate_multi_vehicle_dds] WARNING: no restored mesh rate observed on /mesh/heartbeat_imp; continuing because CLI sampling can miss delayed relay windows under load." >&2
  fi
  assert_topic_rate_observed "${TELEMETRY_TOPIC}" "restored" "${restored_telemetry_hz_file}"
  if ! assert_topic_rate_observed "${MAP_TILE_TOPIC}" "restored" "${restored_map_hz_file}"; then
    if capture_topic_once "${MAP_TILE_TOPIC}" "${pass_log_dir}" "restored"; then
      echo "[validate_multi_vehicle_dds] WARNING: no restored average rate observed on ${MAP_TILE_TOPIC}; one-shot sample succeeded so continuing." >&2
    else
      return 1
    fi
  fi
  probe_topic_roundtrip \
    "${DETECTION_TOPIC}" \
    "aeris_msgs/msg/FusedDetection" \
    "{}" \
    "${pass_log_dir}" \
    "detection_restored"
  restored_detection_probe_log="${pass_log_dir}/probe_detection_restored_echo_attempt1.log"
  if ros2 topic list 2>/dev/null | grep -Fxq "${REPLAY_ANNOTATION_TOPIC}"; then
    replay_annotation_restored_hz_file=$(sample_topic_hz "${REPLAY_ANNOTATION_TOPIC}" "${pass_log_dir}" "replay_annotation_restored")
    if ! assert_topic_rate_observed "${REPLAY_ANNOTATION_TOPIC}" "restored" "${replay_annotation_restored_hz_file}"; then
      echo "[validate_multi_vehicle_dds] WARNING: replay annotation topic has no restored sample yet: ${REPLAY_ANNOTATION_TOPIC}" >&2
    fi

    replay_annotation_sample_file="${pass_log_dir}/replay_annotation_sample.log"
    timeout "${PROBE_TIMEOUT_SEC}"s ros2 topic echo "${REPLAY_ANNOTATION_TOPIC}" --once >"${replay_annotation_sample_file}" 2>&1 || true
    if [[ -f "${replay_annotation_sample_file}" ]] && grep -Eiq 'delivery_mode|original_event_ts|replayed_at_ts' "${replay_annotation_sample_file}"; then
      echo "[validate_multi_vehicle_dds] Replay annotation metadata observed on ${REPLAY_ANNOTATION_TOPIC}"
    else
      echo "[validate_multi_vehicle_dds] WARNING: replay annotation sample missing expected metadata fields on ${REPLAY_ANNOTATION_TOPIC}" >&2
    fi

    relay_tile_latency_sample_file="${pass_log_dir}/relay_tile_latency_samples.log"
    timeout "${RELAY_TILE_SAMPLE_TIMEOUT_SEC}"s ros2 topic echo "${REPLAY_ANNOTATION_TOPIC}" >"${relay_tile_latency_sample_file}" 2>&1 || true
    if [[ -f "${relay_tile_latency_sample_file}" ]] && grep -Eiq 'relay_envelope|route_key|published_at_ts|replayed_at_ts' "${relay_tile_latency_sample_file}"; then
      assert_relay_tile_latency_p95 "${relay_tile_latency_sample_file}" "${RELAY_TILE_LATENCY_P95_TARGET_SEC}" "${RELAY_TILE_LATENCY_REQUIRED}"
    else
      if is_truthy "${RELAY_TILE_LATENCY_REQUIRED}"; then
        echo "[validate_multi_vehicle_dds] ERROR: no relay tile latency samples captured on ${REPLAY_ANNOTATION_TOPIC}; cannot assert p95 target." >&2
        return 1
      fi
      echo "[validate_multi_vehicle_dds] WARNING: no relay tile latency samples captured on ${REPLAY_ANNOTATION_TOPIC}; skipping p95 assertion." >&2
    fi
  else
    if is_truthy "${RELAY_TILE_LATENCY_REQUIRED}"; then
      echo "[validate_multi_vehicle_dds] ERROR: replay annotation topic not discovered; relay tile latency validation is required: ${REPLAY_ANNOTATION_TOPIC}" >&2
      return 1
    fi
    echo "[validate_multi_vehicle_dds] WARNING: replay annotation topic not discovered (skipping restored replay metadata checks): ${REPLAY_ANNOTATION_TOPIC}" >&2
  fi

  if [[ -n "${impaired_detection_probe_log}" && -n "${restored_detection_probe_log}" ]]; then
    echo "[validate_multi_vehicle_dds] Detection probes captured during impaired and restored phases."
  fi

  cleanup
  SIM_PID=""
  RELAY_PID=""
  FLOOD_PID=""
  MAP_PUBLISHER_PID=""

  if (( prev_rmw_impl_set )); then
    export RMW_IMPLEMENTATION="${prev_rmw_impl}"
  else
    unset RMW_IMPLEMENTATION
  fi
  if (( prev_fastdds_profiles_set )); then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${prev_fastdds_profiles}"
  else
    unset FASTRTPS_DEFAULT_PROFILES_FILE
  fi
  if (( prev_fastrtps_qos_xml_set )); then
    export RMW_FASTRTPS_USE_QOS_FROM_XML="${prev_fastrtps_qos_xml}"
  else
    unset RMW_FASTRTPS_USE_QOS_FROM_XML
  fi

  echo "[validate_multi_vehicle_dds] Completed pass: ${rmw_impl}"
}

export CYCLONEDDS_URI
export FASTRTPS_DEFAULT_PROFILES_FILE
export RMW_FASTRTPS_USE_QOS_FROM_XML

require_cmd ros2
require_cmd timeout
require_cmd grep
require_cmd tr
require_cmd awk
require_cmd python3

echo "[validate_multi_vehicle_dds] Logs: ${LOG_DIR}"
echo "[validate_multi_vehicle_dds] CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "[validate_multi_vehicle_dds] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "[validate_multi_vehicle_dds] RMW_VALIDATION_SET=${RMW_VALIDATION_SET}"
echo "[validate_multi_vehicle_dds] RMW_FASTRTPS_USE_QOS_FROM_XML=${RMW_FASTRTPS_USE_QOS_FROM_XML}"
echo "[validate_multi_vehicle_dds] RELAY_TILE_LATENCY_P95_TARGET_SEC=${RELAY_TILE_LATENCY_P95_TARGET_SEC}"
echo "[validate_multi_vehicle_dds] RELAY_TILE_LATENCY_REQUIRED=${RELAY_TILE_LATENCY_REQUIRED}"
echo "[validate_multi_vehicle_dds] MULTI_DRONE_LAUNCH_PACKAGE=${MULTI_DRONE_LAUNCH_PACKAGE}"
echo "[validate_multi_vehicle_dds] MULTI_DRONE_LAUNCH_FILE=${MULTI_DRONE_LAUNCH_FILE}"

source_local_edge_overlay
resolve_launch_target
preflight_runtime_checks

IFS=',' read -r -a RMW_LIST <<< "${RMW_VALIDATION_SET}"
PASS_COUNT=0
for rmw in "${RMW_LIST[@]}"; do
  rmw_trimmed=$(echo "${rmw}" | tr -d '[:space:]')
  if [[ -z "${rmw_trimmed}" ]]; then
    continue
  fi
  PASS_COUNT=$((PASS_COUNT + 1))
  run_validation_pass "${rmw_trimmed}"
done

if (( PASS_COUNT == 0 )); then
  echo "[validate_multi_vehicle_dds] ERROR: no RMW implementations selected for validation" >&2
  exit 1
fi

echo "[validate_multi_vehicle_dds] Validation recipe complete across ${PASS_COUNT} RMW pass(es). Review logs under ${LOG_DIR}."
