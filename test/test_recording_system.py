"""Tests for the simulation recording and replay system.

This module validates the recording system components including:
- Recording profile configuration and topic selection
- Record script dry-run functionality and metadata generation
- Replay script argument handling and dry-run output
- Launch file availability and rosbag integration
- Documentation completeness

Prerequisites:
    - software/sim/config/recording_profile.yaml exists
    - Python 3.8+ with pytest
    - ROS2 environment for rosbag operations
"""

import json
import subprocess
from pathlib import Path


# Path constants for recording system artifacts
PROFILE = Path('software/sim/config/recording_profile.yaml')
RECORD_SCRIPT = Path('software/sim/tools/record_sim_session.py')
REPLAY_SCRIPT = Path('software/sim/tools/replay_mission.py')
REPLAY_LAUNCH = Path('software/sim/launch/replay_recording.launch.py')
SIM_RECORDING_SPEC = Path('docs/specs/stories/sim_recording.md')


def test_recording_profile_exists_and_topics_present():
    """Validate recording profile contains required topics and metadata.

    Given: A recording profile YAML file exists
    When: The profile is parsed for bag name and topic list
    Then: The bag name prefix is 'aeris_mission' and required topics are present
    """
    assert PROFILE.is_file()
    data = json.loads(PROFILE.read_text())
    assert data['bag_name_prefix'] == 'aeris_mission'
    topics = data['rosbag']['topics']
    assert '/clock' in topics
    assert '/tf' in topics
    assert any('/vehicle/scout1' in topic for topic in topics)


def test_record_recording_dry_run(tmp_path):
    """Validate record script creates metadata in dry-run mode.

    Given: A temporary output directory
    When: The record script is invoked with --dry-run flag
    Then: Session folder and metadata.json are created with correct dry_run flag

    Args:
        tmp_path: Pytest fixture providing a temporary directory path.
    """
    assert RECORD_SCRIPT.is_file()
    out_dir = tmp_path / 'recordings'
    cmd = [
        'python3',
        str(RECORD_SCRIPT),
        '--config',
        str(PROFILE),
        '--output-dir',
        str(out_dir),
        '--dry-run',
    ]
    subprocess.run(cmd, check=True)
    subdirs = list(out_dir.glob('*aeris_mission'))
    assert subdirs, 'expected session folder to be created'
    meta = subdirs[0] / 'metadata.json'
    assert meta.is_file()
    metadata = json.loads(meta.read_text())
    assert metadata['dry_run'] is True
    assert metadata['topics'], 'topics should be persisted in metadata'


def test_replay_script_dry_run(tmp_path):
    """Validate replay script outputs expected commands in dry-run mode.

    Given: A fake bag directory for replay input
    When: The replay script is invoked with --dry-run and camera options
    Then: Output contains rosbag command and camera preset information

    Args:
        tmp_path: Pytest fixture providing a temporary directory path.
    """
    assert REPLAY_SCRIPT.is_file()
    bag_dir = tmp_path / 'fake_bag'
    bag_dir.mkdir()
    cmd = [
        'python3',
        str(REPLAY_SCRIPT),
        '--bag',
        str(bag_dir),
        '--dry-run',
        '--camera-preset',
        'tracking',
        '--camera-target',
        'scout1',
    ]
    result = subprocess.run(cmd, check=True, capture_output=True, text=True)
    assert 'ros2 bag command' in result.stdout
    assert 'tracking' in result.stdout


def test_replay_launch_exists():
    """Validate that the replay launch file exists and contains rosbag play.

    Given: The replay launch file should exist
    When: The launch file is read and searched for rosbag commands
    Then: The file exists and contains the ros2 bag play command
    """
    assert REPLAY_LAUNCH.is_file()
    text = REPLAY_LAUNCH.read_text()
    assert 'ros2", "bag", "play' in text


def test_sim_recording_spec_created():
    """Validate that the recording specification documentation exists and is complete.

    Given: The recording system should have documentation
    When: The spec file is read for MCAP and path references
    Then: The file exists and contains MCAP format and log/recordings path
    """
    assert SIM_RECORDING_SPEC.is_file()
    content = SIM_RECORDING_SPEC.read_text()
    assert 'MCAP' in content
    assert 'log/recordings' in content
