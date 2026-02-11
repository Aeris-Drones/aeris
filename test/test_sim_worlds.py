"""Tests for simulation world and model definitions.

This module validates the Gazebo/Ignition simulation assets including:
- World file existence and directory structure
- Disaster scene model composition and lighting
- Launch file availability for world loading
- Supporting documentation and test assets
- Test box model drivetrain configuration

Prerequisites:
    - software/sim/worlds/ directory contains SDF world files
    - software/sim/models/ directory contains SDF model files
    - Python 3.8+ with pytest
"""

import os
import xml.etree.ElementTree as ET


def _load_world(path):
    """Load and parse an SDF world file.

    Args:
        path: Path to the SDF world file.

    Returns:
        Element: The root element of the parsed XML tree.
    """
    tree = ET.parse(path)
    return tree.getroot()


def _load_model(path):
    """Load and parse an SDF model file.

    Args:
        path: Path to the SDF model file.

    Returns:
        Element: The root element of the parsed XML tree.
    """
    tree = ET.parse(path)
    return tree.getroot()


def test_basic_world_scaffold_exists():
    """Validate that the basic world directory and file exist.

    Given: The simulation worlds directory should exist
    When: The directory and basic_world.sdf file are checked
    Then: Both the directory and file exist
    """
    assert os.path.isdir('software/sim/worlds')
    assert os.path.isfile('software/sim/worlds/basic_world.sdf')


def test_disaster_scene_assets_present():
    """Validate disaster scene contains required models and lighting.

    Given: The disaster scene world file exists
    When: The world is parsed for models and lights
    Then: Required building models, debris, and rescue zone are present
          Golden hour and cool fill lights are configured
    """
    world_path = 'software/sim/worlds/disaster_scene.sdf'
    assert os.path.isfile(world_path)
    root = _load_world(world_path)
    models = [m.attrib.get('name') for m in root.findall('.//model')]
    for required in ['building_a_base', 'building_a_top', 'building_b', 'debris_cluster', 'rescue_zone']:
        assert required in models, f"missing model {required}"
    lights = [l.attrib.get('name') for l in root.findall('.//light')]
    assert 'golden_hour_key' in lights
    assert 'cool_fill' in lights


def test_launch_files_exist():
    """Validate that world loading launch files exist.

    Given: The simulation requires launch files for world loading
    When: The launch file paths are checked
    Then: All required launch files exist
    """
    assert os.path.isfile('software/sim/launch/sim_world.launch.py')
    assert os.path.isfile('software/sim/launch/disaster_scene.launch.py')
    assert os.path.isfile('software/sim/launch/multi_drone_sim.launch.py')


def test_supporting_docs_and_scripts_exist():
    """Validate supporting documentation and utility scripts exist.

    Given: The simulation system requires documentation and tools
    When: The doc and script paths are checked
    Then: All required supporting files exist
    """
    assert os.path.isfile('docs/specs/sim_setup.md')
    assert os.path.isfile('docs/specs/disaster_scene.md')
    assert os.path.isfile('docs/specs/stories/multi_drone_sitl.md')
    assert os.path.isfile('software/sim/tools/run_basic_sim.sh')
    assert os.path.isfile('software/sim/models/test_box/model.sdf')


def test_test_box_controls_cmd_vel():
    """Validate the test box model has differential drive and wheel joints.

    Given: The test box model SDF file exists
    When: The model is parsed for plugins and joints
    Then: A differential drive plugin is present
          Left and right wheel joints are defined
    """
    root = _load_model('software/sim/models/test_box/model.sdf')
    plugins = [p.attrib.get('filename', '') for p in root.findall('.//plugin')]
    assert any('diff-drive' in fname for fname in plugins)
    joints = [j.attrib.get('name') for j in root.findall('.//joint')]
    assert 'left_wheel_joint' in joints
    assert 'right_wheel_joint' in joints
