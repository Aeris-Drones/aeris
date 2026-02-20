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


def _parse_cyclonedds_topics() -> dict[str, dict[str, str]]:
    ns = {"c": "https://cdds.io/config"}
    root = ET.parse(CYCLONEDDS_CONFIG).getroot()
    topics: dict[str, dict[str, str]] = {}
    for topic in root.findall(".//c:QoS/c:Topic", ns):
        name = topic.attrib.get("name")
        if not name:
            continue
        reliability = topic.findtext("c:Reliability", default="", namespaces=ns)
        history_node = topic.find("c:History", ns)
        history_kind = history_node.attrib.get("kind", "") if history_node is not None else ""
        history_depth = history_node.attrib.get("depth", "") if history_node is not None else ""
        topics[name] = {
            "reliability": reliability,
            "history_kind": history_kind,
            "history_depth": history_depth,
        }
    return topics


def _parse_fastdds_profiles() -> dict[str, dict[str, str]]:
    ns = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    root = ET.parse(FASTDDS_CONFIG).getroot()
    profiles: dict[str, dict[str, str]] = {}

    for tag in ("publisher", "subscriber"):
        for node in root.findall(f".//f:{tag}", ns):
            profile_name = node.attrib.get("profile_name")
            if not profile_name:
                continue
            reliability = node.findtext(".//f:reliabilityQos/f:kind", default="", namespaces=ns)
            history_depth = node.findtext(".//f:historyQos/f:depth", default="", namespaces=ns)
            profiles[profile_name] = {
                "endpoint": tag,
                "reliability": reliability,
                "history_depth": history_depth,
            }
    return profiles


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


def test_cyclonedds_profiles_cover_control_telemetry_and_video_metadata_classes() -> None:
    topics = _parse_cyclonedds_topics()

    assert topics["orchestrator/heartbeat"]["reliability"] == "Reliable"
    assert topics["map/tiles"]["reliability"] == "Reliable"
    assert topics["mesh/heartbeat_imp"]["reliability"] == "Reliable"

    assert topics["*/stereo/left/image_raw"]["reliability"] == "BestEffort"
    assert topics["*/stereo/right/image_raw"]["reliability"] == "BestEffort"
    assert topics["video/*"]["reliability"] == "BestEffort"
    assert topics["metadata/*"]["reliability"] == "BestEffort"

    assert topics["*"]["reliability"] == "Reliable"


def test_fastdds_profiles_include_parity_for_control_and_video_classes() -> None:
    profiles = _parse_fastdds_profiles()

    assert profiles["control_telemetry_writer"]["endpoint"] == "publisher"
    assert profiles["control_telemetry_writer"]["reliability"] == "RELIABLE_RELIABILITY_QOS"
    assert profiles["control_telemetry_writer"]["history_depth"] == "20"

    assert profiles["control_telemetry_reader"]["endpoint"] == "subscriber"
    assert profiles["control_telemetry_reader"]["reliability"] == "RELIABLE_RELIABILITY_QOS"
    assert profiles["control_telemetry_reader"]["history_depth"] == "20"

    assert profiles["video_metadata_writer"]["endpoint"] == "publisher"
    assert profiles["video_metadata_writer"]["reliability"] == "BEST_EFFORT_RELIABILITY_QOS"
    assert profiles["video_metadata_writer"]["history_depth"] == "5"

    assert profiles["video_metadata_reader"]["endpoint"] == "subscriber"
    assert profiles["video_metadata_reader"]["reliability"] == "BEST_EFFORT_RELIABILITY_QOS"
    assert profiles["video_metadata_reader"]["history_depth"] == "5"


def test_dds_socket_buffers_align_with_sysctl_overlay_limits() -> None:
    cyclone_root = ET.parse(CYCLONEDDS_CONFIG).getroot()
    cyclone_ns = {"c": "https://cdds.io/config"}

    cyclone_receive_buffer = int(
        cyclone_root.findtext(".//c:Internal/c:ReceiveBufferSize", default="0", namespaces=cyclone_ns)
    )
    cyclone_transmit_buffer = int(
        cyclone_root.findtext(".//c:Internal/c:TransmitBufferSize", default="0", namespaces=cyclone_ns)
    )

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
    assert cyclone_receive_buffer == fastdds_receive_buffer
    assert cyclone_transmit_buffer == fastdds_send_buffer

    assert cyclone_receive_buffer <= sysctl["net.core.rmem_max"]
    assert cyclone_transmit_buffer <= sysctl["net.core.wmem_max"]
    assert cyclone_receive_buffer == sysctl["net.core.rmem_default"]
    assert cyclone_transmit_buffer == sysctl["net.core.wmem_default"]


def test_multi_drone_launch_wires_dds_env_vars_for_launched_processes() -> None:
    text = MULTI_DRONE_LAUNCH.read_text()

    assert "DeclareLaunchArgument(\n            'cyclonedds_uri'" in text
    assert "DeclareLaunchArgument(\n            'fastdds_profiles_file'" in text
    assert "DeclareLaunchArgument(\n            'rmw_implementation'" in text

    assert "'CYCLONEDDS_URI': cyclonedds_uri" in text
    assert "'FASTRTPS_DEFAULT_PROFILES_FILE': fastdds_profiles_file" in text
    assert "'RMW_IMPLEMENTATION': rmw_implementation" in text
    assert text.count("additional_env=dds_env") >= 3


def test_validation_recipe_contains_nominal_and_impaired_checks() -> None:
    text = VALIDATION_RECIPE.read_text()

    assert "ros2 launch aeris_sim multi_drone_sim.launch.py" in text
    assert "cyclonedds_uri:=\"${CYCLONEDDS_URI}\"" in text
    assert "fastdds_profiles_file:=\"${FASTRTPS_DEFAULT_PROFILES_FILE}\"" in text
    assert "rmw_implementation:=\"${RMW_IMPLEMENTATION}\"" in text

    assert "wait_for_namespace \"scout1\"" in text
    assert "wait_for_namespace \"scout2\"" in text
    assert "wait_for_namespace \"ranger1\"" in text

    assert "ros2 topic info -v /map/tiles" in text
    assert "sample_topic_hz \"/map/tiles\" \"nominal\"" in text
    assert "ros2 run aeris_tools dds_flood_tester" in text

    assert "ros2 run aeris_mesh_agent impairment_relay" in text
    assert "ros2 param set /impairment_relay drop_prob" in text
    assert "ros2 param set /impairment_relay delay_ms" in text

    assert "sample_topic_hz \"/mesh/heartbeat_imp\" \"impaired\"" in text
    assert "sample_topic_hz \"/mesh/heartbeat_imp\" \"restored\"" in text


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
