"""Tests for multi-drone simulation configuration validation.

This module validates the multi-drone simulation setup including:
- Configuration file structure and syntax
- Vehicle name and port uniqueness constraints
- Minimum spacing requirements between vehicles
- Supporting asset availability

Prerequisites:
    - software/sim/config/multi_drone.yaml exists and is valid YAML
    - Python 3.8+ with pytest
"""

import json
import math
import os
from pathlib import Path


# Path to the multi-drone configuration file
CONFIG_PATH = Path('software/sim/config/multi_drone.yaml')


def load_config():
    """Load and parse the multi-drone configuration file.

    Returns:
        list: List of vehicle configuration dictionaries.
    """
    return json.loads(CONFIG_PATH.read_text())['vehicles']


def test_config_file_exists():
    """Validate that the multi-drone configuration file exists.

    Given: The repository contains multi-drone configuration
    When: The configuration file path is checked
    Then: The file exists at the expected location
    """
    assert CONFIG_PATH.is_file()


def test_vehicle_ports_and_names_unique():
    """Validate that all vehicles have unique names and MAVLink ports.

    Given: A list of vehicle configurations is loaded
    When: Vehicle names and MAVLink UDP ports are extracted
    Then: All names are unique and all ports are unique
    """
    vehicles = load_config()
    names = set()
    ports = set()
    for v in vehicles:
        assert v['name'] not in names
        names.add(v['name'])
        assert v['mavlink_udp_port'] not in ports
        ports.add(v['mavlink_udp_port'])


def test_vehicle_spacing_minimum_met():
    """Validate minimum distance between vehicle spawn positions.

    Given: A list of vehicle configurations with pose data
    When: Pairwise distances between vehicles are calculated
    Then: All vehicle pairs are separated by at least 50 meters

    Raises:
        AssertionError: If any two vehicles are closer than 50 meters.
    """
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
    """Validate that required supporting files for multi-drone simulation exist.

    Given: The multi-drone simulation requires launcher and launch files
    When: The supporting asset paths are checked
    Then: All required files exist at their expected locations
    """
    assert os.path.isfile('software/sim/tools/run_multi_drone_sitl.py')
    assert os.path.isfile('software/sim/launch/multi_drone_sim.launch.py')
