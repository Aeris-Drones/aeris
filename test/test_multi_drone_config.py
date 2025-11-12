import json
import math
import os
from pathlib import Path


CONFIG_PATH = Path('software/sim/config/multi_drone.yaml')


def load_config():
    return json.loads(CONFIG_PATH.read_text())['vehicles']


def test_config_file_exists():
    assert CONFIG_PATH.is_file()


def test_vehicle_ports_and_names_unique():
    vehicles = load_config()
    names = set()
    ports = set()
    for v in vehicles:
        assert v['name'] not in names
        names.add(v['name'])
        assert v['mavlink_udp_port'] not in ports
        ports.add(v['mavlink_udp_port'])


def test_vehicle_spacing_minimum_met():
    vehicles = load_config()
    poses = []
    for v in vehicles:
        xyz = [float(n) for n in v['pose'].split()[0:3]]
        poses.append((v['name'], xyz))
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            a, posa = poses[i]
            b, posb = poses[j]
            assert math.dist(posa, posb) >= 50.0, f"{a}-{b} too close"


def test_supporting_assets_present():
    assert os.path.isfile('software/sim/tools/run_multi_drone_sitl.py')
    assert os.path.isfile('software/sim/launch/multi_drone_sim.launch.py')
