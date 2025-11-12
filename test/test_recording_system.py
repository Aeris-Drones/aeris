import json
import subprocess
from pathlib import Path

PROFILE = Path('software/sim/config/recording_profile.yaml')
RECORD_SCRIPT = Path('software/sim/tools/record_sim_session.py')
REPLAY_SCRIPT = Path('software/sim/tools/replay_mission.py')
REPLAY_LAUNCH = Path('software/sim/launch/replay_recording.launch.py')
SIM_RECORDING_SPEC = Path('docs/specs/stories/sim_recording.md')


def test_recording_profile_exists_and_topics_present():
    assert PROFILE.is_file()
    data = json.loads(PROFILE.read_text())
    assert data['bag_name_prefix'] == 'aeris_mission'
    topics = data['rosbag']['topics']
    assert '/clock' in topics
    assert '/tf' in topics
    assert any('/vehicle/scout1' in topic for topic in topics)


def test_record_recording_dry_run(tmp_path):
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
    assert REPLAY_LAUNCH.is_file()
    text = REPLAY_LAUNCH.read_text()
    assert 'ros2", "bag", "play' in text


def test_sim_recording_spec_created():
    assert SIM_RECORDING_SPEC.is_file()
    content = SIM_RECORDING_SPEC.read_text()
    assert 'MCAP' in content
    assert 'log/recordings' in content
