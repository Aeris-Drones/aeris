"""Tests for the cinematic camera system integration.

This module validates the camera system components including:
- SDF model definitions and plugin configurations
- ROS message and service interfaces
- Camera path building and waypoint tools
- World scene integration

Prerequisites:
    - ROS2 environment with aeris_msgs package built
    - Python 3.8+ with pytest
    - Access to software/sim/models and software/edge directories
"""

import json
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


# Path constants for camera system artifacts
CAMERA_MODEL = Path('software/sim/models/cinematic_camera/model.sdf')
CAMERA_DOC = Path('docs/specs/stories/cinematic_camera.md')
CAMERA_SRV = Path('software/edge/src/aeris_msgs/srv/SetCameraView.srv')
CAMERA_MSG = Path('software/edge/src/aeris_msgs/msg/CameraWaypoint.msg')
BUILDER = Path('software/sim/tools/camera_path_builder.py')
SENDER = Path('software/sim/tools/send_camera_view.py')
DISASTER_SCENE = Path('software/sim/worlds/disaster_scene.sdf')


def test_cinematic_camera_model_has_plugin():
    """Validate that the cinematic camera model includes the aeris controller plugin.

    Given: A cinematic camera SDF model file exists
    When: The model is parsed for plugin definitions
    Then: The aeris_camera_controller plugin is present
    """
    assert CAMERA_MODEL.is_file()
    root = ET.parse(CAMERA_MODEL).getroot()
    plugins = [plug.attrib.get('filename', '') for plug in root.findall('.//plugin')]
    assert any('aeris_camera_controller' in name for name in plugins)


def test_disaster_scene_includes_camera_rig():
    """Verify the disaster scene world includes the cinematic camera model.

    Given: A disaster scene SDF world file exists
    When: The world is parsed for included models
    Then: The cinematic_camera model URI is present in includes
    """
    root = ET.parse(DISASTER_SCENE).getroot()
    include_uris = [node.findtext('uri') for node in root.findall('.//include')]
    assert 'model://cinematic_camera' in include_uris


def test_interfaces_defined():
    """Validate ROS message and service definitions for camera waypoints.

    Given: Camera message and service definition files exist
    When: The definitions are parsed for required fields
    Then: CameraWaypoint.msg contains pose and FOV fields
          SetCameraView.srv contains path and preset fields
    """
    assert CAMERA_MSG.is_file()
    msg_text = CAMERA_MSG.read_text()
    assert 'geometry_msgs/Pose pose' in msg_text
    assert 'float32 field_of_view_deg' in msg_text

    assert CAMERA_SRV.is_file()
    srv_text = CAMERA_SRV.read_text()
    assert 'CameraWaypoint[] path' in srv_text
    assert 'string preset_name' in srv_text


def test_cinematic_docs_and_tools_exist(tmp_path):
    """Validate camera tools are executable and produce valid output.

    Given: Camera documentation and tool scripts exist with execute permissions
    When: The camera_path_builder tool is invoked with test keyframes
    Then: Valid JSON output is produced with correct waypoint metadata

    Args:
        tmp_path: Pytest fixture providing a temporary directory path.

    Returns:
        None
    """
    assert CAMERA_DOC.is_file()
    assert BUILDER.is_file()
    assert SENDER.is_file()
    assert os.access(BUILDER, os.X_OK)
    assert os.access(SENDER, os.X_OK)

    output = tmp_path / 'path.json'
    subprocess.run(
        [
            'python3',
            str(BUILDER),
            '--output',
            str(output),
            '--keyframe',
            '0,-60,0,70,0,-0.5,0,55',
            '--keyframe',
            '6,10,20,40,0,-0.3,1.2,40',
        ],
        check=True,
    )
    data = json.loads(output.read_text())
    assert data['metadata']['version'] == 1
    assert len(data['waypoints']) == 2
    assert data['waypoints'][0]['position']['z'] == 70
