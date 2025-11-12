import json
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


CAMERA_MODEL = Path('software/sim/models/cinematic_camera/model.sdf')
CAMERA_DOC = Path('docs/specs/stories/cinematic_camera.md')
CAMERA_SRV = Path('software/edge/src/aeris_msgs/srv/SetCameraView.srv')
CAMERA_MSG = Path('software/edge/src/aeris_msgs/msg/CameraWaypoint.msg')
BUILDER = Path('software/sim/tools/camera_path_builder.py')
SENDER = Path('software/sim/tools/send_camera_view.py')
DISASTER_SCENE = Path('software/sim/worlds/disaster_scene.sdf')


def test_cinematic_camera_model_has_plugin():
    assert CAMERA_MODEL.is_file()
    root = ET.parse(CAMERA_MODEL).getroot()
    plugins = [plug.attrib.get('filename', '') for plug in root.findall('.//plugin')]
    assert any('aeris_camera_controller' in name for name in plugins)


def test_disaster_scene_includes_camera_rig():
    root = ET.parse(DISASTER_SCENE).getroot()
    include_uris = [node.findtext('uri') for node in root.findall('.//include')]
    assert 'model://cinematic_camera' in include_uris


def test_interfaces_defined():
    assert CAMERA_MSG.is_file()
    msg_text = CAMERA_MSG.read_text()
    assert 'geometry_msgs/Pose pose' in msg_text
    assert 'float32 field_of_view_deg' in msg_text

    assert CAMERA_SRV.is_file()
    srv_text = CAMERA_SRV.read_text()
    assert 'CameraWaypoint[] path' in srv_text
    assert 'string preset_name' in srv_text


def test_cinematic_docs_and_tools_exist(tmp_path):
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
