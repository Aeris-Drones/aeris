import os
import xml.etree.ElementTree as ET


def _load_world(path):
    tree = ET.parse(path)
    return tree.getroot()


def test_basic_world_scaffold_exists():
    assert os.path.isdir('software/sim/worlds')
    assert os.path.isfile('software/sim/worlds/basic_world.sdf')


def test_disaster_scene_assets_present():
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
    assert os.path.isfile('software/sim/launch/sim_world.launch.py')
    assert os.path.isfile('software/sim/launch/disaster_scene.launch.py')


def test_supporting_docs_and_scripts_exist():
    assert os.path.isfile('docs/specs/sim_setup.md')
    assert os.path.isfile('docs/specs/disaster_scene.md')
    assert os.path.isfile('software/sim/tools/run_basic_sim.sh')
    assert os.path.isfile('software/sim/models/test_box/model.sdf')
