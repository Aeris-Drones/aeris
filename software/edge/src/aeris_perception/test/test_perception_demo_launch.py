from pathlib import Path


def test_perception_demo_launch_wires_rgb_ingest_node() -> None:
    launch_file = (
        Path(__file__).resolve().parents[3]
        / "launch"
        / "perception_demo.launch.py"
    )
    content = launch_file.read_text()

    assert 'executable="rgb_ingest"' in content
    assert "rgb_source_type" in content
    assert "rgb_source_uri" in content
    assert "perception_demo_rgb_ingest.yaml" in content
