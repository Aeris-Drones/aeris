import os
from pathlib import Path


LAUNCH_PATH = Path('software/edge/src/aeris_map/launch/rtabmap_vio_sim.launch.py')
OPENVINS_CFG = Path('software/edge/src/aeris_map/config/openvins_sim.yaml')
RTABMAP_CFG = Path('software/edge/src/aeris_map/config/rtabmap_vio_sim.yaml')
PARITY_DOC = Path('software/sim/config/slam_parity.md')
TRAJECTORY_JSON = Path('software/sim/config/loop_closure_path.json')
TOPIC_CHECK_SCRIPT = Path('software/sim/tools/validate_slam_topics.sh')
LOOP_CHECK_SCRIPT = Path('software/sim/tools/validate_loop_closure.sh')


def test_rtabmap_vio_assets_exist():
    assert LAUNCH_PATH.is_file()
    assert OPENVINS_CFG.is_file()
    assert RTABMAP_CFG.is_file()


def test_launch_wires_openvins_and_rtabmap_nodes():
    text = LAUNCH_PATH.read_text()
    assert "package='ov_msckf'" in text
    assert "package='rtabmap_slam'" in text
    assert 'openvins_odom_topic' in text
    assert "DeclareLaunchArgument('base_frame', default_value='base_link')" in text
    assert "DeclareLaunchArgument('map_frame', default_value='map')" in text
    assert "DeclareLaunchArgument('odom_frame', default_value='odom')" in text


def test_configs_include_expected_topic_and_frame_contracts():
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
    assert PARITY_DOC.is_file()
    assert TRAJECTORY_JSON.is_file()

    parity_text = PARITY_DOC.read_text()
    assert 'map -> odom -> base_link' in parity_text
    assert 'No simulation-specific branches' in parity_text


def test_slam_validation_scripts_present_and_executable():
    assert TOPIC_CHECK_SCRIPT.is_file()
    assert LOOP_CHECK_SCRIPT.is_file()
    assert os.access(TOPIC_CHECK_SCRIPT, os.X_OK)
    assert os.access(LOOP_CHECK_SCRIPT, os.X_OK)

    topics_script_text = TOPIC_CHECK_SCRIPT.read_text()
    assert 'ros2 topic list' in topics_script_text
    assert 'ros2 topic hz' in topics_script_text
    assert '/rtabmap/cloud_map' in topics_script_text

    loop_script_text = LOOP_CHECK_SCRIPT.read_text()
    assert 'ros2 topic echo /rtabmap/info' in loop_script_text
    assert 'tf2_echo map base_link' in loop_script_text


def test_run_basic_sim_has_bridge_defaults_for_stereo_and_imu():
    script = Path('software/sim/tools/run_basic_sim.sh').read_text()
    assert 'SCOUT_MODEL_NAME=${SCOUT_MODEL_NAME:-scout1}' in script
    assert 'LEFT_IMAGE_TOPIC_GZ' in script
    assert 'RIGHT_IMAGE_TOPIC_GZ' in script
    assert 'IMU_TOPIC_GZ' in script
    assert 'BRIDGE_REMAP_ARGS' in script


def test_multi_drone_launch_exposes_scout_model_name_override():
    launch = Path('software/sim/launch/multi_drone_sim.launch.py').read_text()
    assert "'scout_model_name'" in launch
    assert 'SCOUT_MODEL_NAME={scout_model_name}' in launch
