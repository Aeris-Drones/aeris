"""Static validation tests for Story 4.1 DDS multi-vehicle communication wiring.

These tests intentionally validate configuration and launch wiring without
requiring a running ROS graph.
"""

from pathlib import Path
import re
import stat
import xml.etree.ElementTree as ET


CYCLONEDDS_CONFIG = Path("software/edge/config/dds/cyclonedds.xml")
FASTDDS_CONFIG = Path("software/edge/config/dds/fastdds.xml")
SYSCTL_OVERLAY = Path("software/edge/config/dds/sysctl_overlay.conf")
MULTI_DRONE_LAUNCH = Path("software/sim/launch/multi_drone_sim.launch.py")
VALIDATION_RECIPE = Path("software/sim/tools/validate_multi_vehicle_dds.sh")


def _parse_sysctl_values() -> dict[str, int]:
    values: dict[str, int] = {}
    for line in SYSCTL_OVERLAY.read_text().splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, raw_value = [part.strip() for part in stripped.split("=", 1)]
        if " " in raw_value:
            continue
        try:
            values[key] = int(raw_value)
        except ValueError:
            continue
    return values


def test_cyclonedds_config_uses_humble_compatible_elements() -> None:
    ns = {"c": "https://cdds.io/config"}
    root = ET.parse(CYCLONEDDS_CONFIG).getroot()

    network_if = root.find(".//c:General/c:Interfaces/c:NetworkInterface", ns)
    assert network_if is not None
    assert network_if.attrib.get("autodetermine") == "true"
    assert root.findtext(".//c:General/c:AllowMulticast", default="", namespaces=ns) == "true"
    assert root.findtext(".//c:General/c:MaxMessageSize", default="", namespaces=ns) == "131072B"

    assert root.findtext(".//c:Discovery/c:ParticipantIndex", default="", namespaces=ns) == "auto"
    assert root.findtext(".//c:Discovery/c:MaxAutoParticipantIndex", default="", namespaces=ns) == "32"
    assert root.find(".//c:Discovery/c:Peers/c:Peer", ns) is not None

    # Per-topic QoS and low-level internal tuning are handled by FastDDS profiles in this story.
    assert root.find(".//c:QoS", ns) is None
    assert root.find(".//c:Internal", ns) is None


def test_fastdds_config_uses_humble_compatible_elements() -> None:
    fastdds_root = ET.parse(FASTDDS_CONFIG).getroot()
    fastdds_ns = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}

    assert (
        fastdds_root.findtext(
            ".//f:transport_descriptor/f:transport_id", default="", namespaces=fastdds_ns
        )
        == "udp_ipv4"
    )
    assert (
        fastdds_root.findtext(".//f:transport_descriptor/f:type", default="", namespaces=fastdds_ns)
        == "UDPv4"
    )
    assert (
        fastdds_root.findtext(
            ".//f:participant[@profile_name='aeris_participant']/f:rtps/f:userTransports/f:transport_id",
            default="",
            namespaces=fastdds_ns,
        )
        == "udp_ipv4"
    )
    assert (
        fastdds_root.findtext(
            ".//f:participant[@profile_name='aeris_participant']/f:rtps/f:useBuiltinTransports",
            default="",
            namespaces=fastdds_ns,
        )
        == "false"
    )

    # Legacy tags used by older Fast DDS schema revisions must not be present.
    assert fastdds_root.find(".//f:reliabilityQos", fastdds_ns) is None
    assert fastdds_root.find(".//f:publishModeQos", fastdds_ns) is None


def test_fastdds_socket_buffers_align_with_sysctl_overlay_limits() -> None:
    fastdds_root = ET.parse(FASTDDS_CONFIG).getroot()
    fastdds_ns = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    fastdds_send_buffer = int(
        fastdds_root.findtext(
            ".//f:transport_descriptor/f:sendBufferSize", default="0", namespaces=fastdds_ns
        )
    )
    fastdds_receive_buffer = int(
        fastdds_root.findtext(
            ".//f:transport_descriptor/f:receiveBufferSize", default="0", namespaces=fastdds_ns
        )
    )

    sysctl = _parse_sysctl_values()
    assert fastdds_receive_buffer > 0
    assert fastdds_send_buffer > 0
    assert fastdds_receive_buffer <= sysctl["net.core.rmem_max"]
    assert fastdds_send_buffer <= sysctl["net.core.wmem_max"]
    assert fastdds_receive_buffer == sysctl["net.core.rmem_default"]
    assert fastdds_send_buffer == sysctl["net.core.wmem_default"]


def test_multi_drone_launch_wires_dds_env_vars_for_launched_processes() -> None:
    text = MULTI_DRONE_LAUNCH.read_text()

    assert "DeclareLaunchArgument(\n            'cyclonedds_uri'" in text
    assert "DeclareLaunchArgument(\n            'fastdds_profiles_file'" in text
    assert "DeclareLaunchArgument(\n            'rmw_implementation'" in text
    assert "DeclareLaunchArgument(\n            'rmw_fastrtps_use_qos_from_xml'" in text

    assert "'CYCLONEDDS_URI': cyclonedds_uri" in text
    assert "'FASTRTPS_DEFAULT_PROFILES_FILE': fastdds_profiles_file" in text
    assert "'RMW_IMPLEMENTATION': rmw_implementation" in text
    assert "'RMW_FASTRTPS_USE_QOS_FROM_XML': rmw_fastrtps_use_qos_from_xml" in text
    assert text.count("additional_env=dds_env") >= 3


def test_validation_recipe_contains_nominal_and_impaired_checks() -> None:
    text = VALIDATION_RECIPE.read_text()

    assert "RMW_VALIDATION_SET=\"rmw_cyclonedds_cpp,rmw_fastrtps_cpp\"" in text
    assert "run_validation_pass \"${rmw_trimmed}\"" in text
    assert "MULTI_DRONE_LAUNCH_PACKAGE=${MULTI_DRONE_LAUNCH_PACKAGE:-aeris_sim}" in text
    assert "MULTI_DRONE_LAUNCH_FILE=${MULTI_DRONE_LAUNCH_FILE:-" in text
    assert "resolve_launch_target" in text
    assert "LAUNCH_CMD=(\"ros2\" \"launch\" \"${MULTI_DRONE_LAUNCH_PACKAGE}\" \"multi_drone_sim.launch.py\")" in text
    assert "LAUNCH_CMD=(\"ros2\" \"launch\" \"${MULTI_DRONE_LAUNCH_FILE}\")" in text
    assert "\"${LAUNCH_CMD[@]}\" \"${launch_args[@]}\"" in text
    assert "cyclonedds_uri:=${CYCLONEDDS_URI}" in text
    assert "fastdds_profiles_file:=${fastdds_profiles_for_pass}" in text
    assert "rmw_implementation:=${rmw_impl}" in text
    assert "rmw_fastrtps_use_qos_from_xml:=${fastrtps_qos_xml_for_pass}" in text

    assert "wait_for_topic \"${TELEMETRY_TOPIC}\"" in text
    assert "wait_for_topic \"${MAP_TILE_TOPIC}\"" in text
    assert "wait_for_topic \"${VIDEO_METADATA_TOPIC}\"" in text
    assert "probe_topic_roundtrip" in text
    assert "PROBE_TIMEOUT_SEC=${PROBE_TIMEOUT_SEC:-20}" in text
    assert "PROBE_ATTEMPTS=${PROBE_ATTEMPTS:-3}" in text
    assert "PROBE_PUB_RATE_HZ=${PROBE_PUB_RATE_HZ:-5}" in text
    assert "PROBE_PUB_COUNT=${PROBE_PUB_COUNT:-15}" in text
    assert "for (( attempt = 1; attempt <= PROBE_ATTEMPTS; attempt++ ))" in text
    assert "-r \"${PROBE_PUB_RATE_HZ}\"" in text
    assert "-t \"${PROBE_PUB_COUNT}\"" in text
    assert "REQUIRED_NAMESPACES=${REQUIRED_NAMESPACES:-scout1,scout2}" in text
    assert "wait_for_namespace \"${ns_trimmed}\"" in text

    assert "capture_topic_info \"${TELEMETRY_TOPIC}\"" in text
    assert "capture_topic_info \"${VIDEO_METADATA_TOPIC}\"" in text
    assert "assert_topic_info_reliability \"${VIDEO_METADATA_TOPIC}\" \"RELIABLE|BEST[_ ]?EFFORT\"" in text
    assert "sample_topic_hz \"${TELEMETRY_TOPIC}\" \"${pass_log_dir}\" \"impaired\"" in text

    assert "python3 \"${REPO_ROOT}/software/edge/tools/dds_flood_tester.py\"" in text
    assert "ros2 run aeris_tools dds_flood_tester" not in text

    assert "ros2 run aeris_mesh_agent impairment_relay" in text
    assert "ros2 param set /impairment_relay drop_prob" in text
    assert "ros2 param set /impairment_relay delay_ms" in text

    assert "sample_topic_hz \"/mesh/heartbeat_imp\" \"${pass_log_dir}\" \"impaired\"" in text
    assert "sample_topic_hz \"/mesh/heartbeat_imp\" \"${pass_log_dir}\" \"restored\"" in text
    assert "preflight_runtime_checks" in text
    assert "ensure_local_ros_package_visible" in text
    assert "ensure_local_ros_package_visible aeris_msgs || true" in text
    assert "ensure_local_ros_package_visible aeris_map || true" in text
    assert "ros2 run aeris_map map_tile_publisher" in text
    assert "python3 -c \"import pymavlink\"" in text
    assert "ERROR: runtime preflight checks failed; aborting validation early." in text


def test_validation_recipe_is_executable() -> None:
    mode = VALIDATION_RECIPE.stat().st_mode
    assert mode & stat.S_IXUSR


def test_validation_recipe_references_required_diagnostic_commands() -> None:
    text = VALIDATION_RECIPE.read_text()

    command_patterns = (
        r"\bros2\s+topic\s+info\s+-v\b",
        r"\bros2\s+topic\s+hz\b",
        r"\bros2\s+param\s+set\b",
        r"\bdds_flood_tester\b",
    )

    for pattern in command_patterns:
        assert re.search(pattern, text), f"Missing expected command pattern: {pattern}"
