"""Tests for RTAB-Map VIO (Visual-Inertial Odometry) simulation integration.

This module validates the SLAM stack integration including:
- Launch file configuration for OpenVINS and RTAB-Map nodes
- Configuration file topic and frame ID contracts
- Validation script availability and correctness
- Trajectory validation for loop closure testing
- Utility function testing for position extraction and rate parsing

Prerequisites:
    - software/edge/src/aeris_map/ configuration files exist
    - Python 3.8+ with pytest
    - ROS2 environment with SLAM packages
"""

import os
import sys
from pathlib import Path


# Add tools directory to path for importing validation utilities
TOOLS_DIR = Path('software/sim/tools').resolve()
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from validation_utils import (  # noqa: E402
    Point3D,
    extract_odom_positions,
    has_loop_closure_signal,
    is_closed_loop,
    is_pose_near,
    load_trajectory_waypoints,
    parse_average_rate,
)


# Path constants for SLAM configuration and validation artifacts
LAUNCH_PATH = Path('software/edge/src/aeris_map/launch/rtabmap_vio_sim.launch.py')
OPENVINS_CFG = Path('software/edge/src/aeris_map/config/openvins_sim.yaml')
RTABMAP_CFG = Path('software/edge/src/aeris_map/config/rtabmap_vio_sim.yaml')
PARITY_DOC = Path('software/sim/config/slam_parity.md')
TRAJECTORY_JSON = Path('software/sim/config/loop_closure_path.json')
TOPIC_CHECK_SCRIPT = Path('software/sim/tools/validate_slam_topics.sh')
LOOP_CHECK_SCRIPT = Path('software/sim/tools/validate_loop_closure.sh')
RUN_BASIC_SIM_SCRIPT = Path('software/sim/tools/run_basic_sim.sh')
MULTI_DRONE_SITL_STORY_DOC = Path('docs/specs/stories/multi_drone_sitl.md')


def test_rtabmap_vio_assets_exist():
    """Validate that required SLAM configuration files exist.

    Given: The SLAM stack requires launch and config files
    When: The paths for launch file and config files are checked
    Then: All required files exist at their expected locations
    """
    assert LAUNCH_PATH.is_file()
    assert OPENVINS_CFG.is_file()
    assert RTABMAP_CFG.is_file()


def test_launch_wires_openvins_and_rtabmap_nodes():
    """Validate launch file configures OpenVINS and RTAB-Map nodes correctly.

    Given: The launch file exists
    When: The launch file text is searched for node configurations
    Then: OpenVINS and RTAB-Map packages are declared with correct topic mappings
          and frame configuration arguments are present
    """
    text = LAUNCH_PATH.read_text()
    assert "package='ov_msckf'" in text
    assert "package='rtabmap_slam'" in text
    assert 'openvins_odom_topic' in text
    assert "DeclareLaunchArgument('scout_model_name', default_value='scout1')" in text
    assert 'rtabmap_database_path' in text
    assert 'openvins_log_directory' in text
    assert "LaunchConfiguration('scout_model_name')" in text
    assert "DeclareLaunchArgument('base_frame', default_value='base_link')" in text
    assert "DeclareLaunchArgument('map_frame', default_value='map')" in text
    assert "DeclareLaunchArgument('odom_frame', default_value='odom')" in text


def test_configs_include_expected_topic_and_frame_contracts():
    """Validate configuration files define correct topic and frame contracts.

    Given: OpenVINS and RTAB-Map config files exist
    When: The configs are parsed for topic subscriptions and frame IDs
    Then: OpenVINS subscribes to stereo images and IMU with correct output topic
          RTAB-Map defines correct frame IDs and subscription flags
    """
    openvins_text = OPENVINS_CFG.read_text()
    assert '/scout1/stereo/left/image_raw' in openvins_text
    assert '/scout1/stereo/right/image_raw' in openvins_text
    assert '/scout1/imu/data' in openvins_text
    assert 'output_odom_topic: /scout1/openvins/odom' in openvins_text

    rtabmap_text = RTABMAP_CFG.read_text()
    assert 'map_frame_id: map' in rtabmap_text
    assert 'odom_frame_id: odom' in rtabmap_text
    assert 'frame_id: base_link' in rtabmap_text
    assert 'subscribe_stereo: true' in rtabmap_text
    assert 'subscribe_imu: true' in rtabmap_text


def test_parity_docs_and_trajectory_present():
    """Validate SLAM parity documentation and loop closure trajectory exist.

    Given: The SLAM system should have parity docs and test trajectory
    When: The parity document and trajectory file are checked
    Then: Both files exist and parity doc contains frame chain and branch info
    """
    assert PARITY_DOC.is_file()
    assert TRAJECTORY_JSON.is_file()

    parity_text = PARITY_DOC.read_text()
    assert 'map -> odom -> base_link' in parity_text
    assert 'No simulation-specific branches' in parity_text


def test_slam_validation_scripts_present_and_executable():
    """Validate that SLAM validation scripts exist and are executable.

    Given: The SLAM system requires validation scripts
    When: The script paths are checked for existence and execute permission
    Then: All scripts exist and have execute permissions
    """
    assert TOPIC_CHECK_SCRIPT.is_file()
    assert LOOP_CHECK_SCRIPT.is_file()
    assert RUN_BASIC_SIM_SCRIPT.is_file()
    assert os.access(TOPIC_CHECK_SCRIPT, os.X_OK)
    assert os.access(LOOP_CHECK_SCRIPT, os.X_OK)
    assert os.access(RUN_BASIC_SIM_SCRIPT, os.X_OK)


def test_validate_slam_topics_script_enforces_thresholds():
    """Validate the topic check script enforces minimum rate thresholds.

    Given: The topic validation script exists
    When: The script content is searched for threshold constants
    Then: MIN_CAMERA_HZ, MIN_IMU_HZ, and assert_min_rate are defined
    """
    text = TOPIC_CHECK_SCRIPT.read_text()
    assert 'MIN_CAMERA_HZ' in text
    assert 'MIN_IMU_HZ' in text
    assert 'assert_min_rate' in text
    assert 'parse_average_rate' in text


def test_validate_loop_closure_script_is_hard_fail_and_chain_checks():
    """Validate the loop closure script performs hard fail and TF chain checks.

    Given: The loop closure validation script exists
    When: The script content is searched for validation logic
    Then: CLOSURE_TOKEN_REGEX, error messages, and TF echo commands are present
    """
    text = LOOP_CHECK_SCRIPT.read_text()
    assert 'CLOSURE_TOKEN_REGEX' in text
    assert 'no loop-closure signal found' in text
    assert 'tf2_echo map odom' in text
    assert 'tf2_echo odom base_link' in text
    assert 'extract_odom_positions' in text


def test_run_basic_sim_script_supports_custom_bridge_topics_and_correct_repo_root():
    """Validate the basic sim script supports custom bridge topics and repo root.

    Given: The basic sim runner script exists
    When: The script content is searched for bridge topic handling
    Then: REPO_ROOT calculation, BRIDGE_TOPIC_ARGS parsing, and clock topic are present
    """
    text = RUN_BASIC_SIM_SCRIPT.read_text()
    assert 'REPO_ROOT=$(cd -- "${SCRIPT_DIR}/../../.." && pwd)' in text
    assert "IFS=' ' read -r -a BRIDGE_TOPIC_ARGS <<< \"${BRIDGE_TOPICS}\"" in text
    assert 'APPLY_BRIDGE_REMAPS_WITH_CUSTOM_TOPICS' in text
    assert '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' in text


def test_multi_drone_story_doc_uses_execute_flag_for_sitl():
    """Validate the multi-drone story doc references the execute flag.

    Given: The multi-drone SITL story documentation exists
    When: The document is searched for SITL execution commands
    Then: The --execute flag and run_multi_drone_sitl.py command are present
    """
    text = MULTI_DRONE_SITL_STORY_DOC.read_text()
    assert '--execute' in text
    assert 'run_multi_drone_sitl.py --config software/sim/config/multi_drone.yaml --execute' in text


def test_parse_average_rate_uses_latest_sample():
    """Validate parse_average_rate extracts the most recent rate value.

    Given: A multi-line rostopic hz output with multiple rate samples
    When: parse_average_rate is called with the output
    Then: The most recent rate (15.2) is returned, or None for invalid input
    """
    output = """
average rate: 14.8
min: 14.6 max: 15.1 std dev: 0.2 window: 30
average rate: 15.2
"""
    assert parse_average_rate(output) == 15.2
    assert parse_average_rate("header only") is None


def test_loop_closure_signal_detection():
    """Validate has_loop_closure_signal correctly identifies loop closure patterns.

    Given: Log lines containing loop closure and non-loop-closure messages
    When: has_loop_closure_signal is called with various patterns
    Then: True is returned for matching lines, False otherwise
    """
    assert has_loop_closure_signal("Loop closure detected", r"loop|closure")
    assert not has_loop_closure_signal("No constraints yet", r"loop closure")


def test_trajectory_is_valid_closed_loop():
    """Validate the test trajectory forms a valid closed loop.

    Given: A trajectory JSON file with waypoint data
    When: Waypoints are loaded and checked for loop closure
    Then: At least 2 waypoints exist and start/end are within tolerance
    """
    waypoints = load_trajectory_waypoints(TRAJECTORY_JSON)
    assert len(waypoints) >= 2
    assert is_closed_loop(waypoints, tolerance_m=0.01)


def test_extract_odom_positions_and_endpoint_matching():
    """Validate extract_odom_positions parses positions and endpoint matching works.

    Given: A YAML-formatted odometry output with multiple poses
    When: Positions are extracted and compared to expected endpoint
    Then: All 3 positions are extracted and final pose matches within tolerance
    """
    sample = """
pose:
  pose:
    position:
      x: -35.0
      y: 0.0
      z: 8.0
---
pose:
  pose:
    position:
      x: -34.2
      y: 0.5
      z: 8.1
---
pose:
  pose:
    position:
      x: -35.1
      y: -0.1
      z: 8.0
"""
    points = extract_odom_positions(sample)
    assert len(points) == 3
    assert is_pose_near(points[-1], Point3D(-35.0, 0.0, 8.0), tolerance_m=0.25)
