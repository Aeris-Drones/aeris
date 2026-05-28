"""Launch file for perception pipeline demonstration.

Spawns synthetic sensor publishers (thermal, acoustic, gas) and mesh network
impairment shims to simulate degraded communication conditions for bench testing.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the perception demo.

    Returns:
        LaunchDescription: A launch description containing all nodes for the
        perception demonstration, including thermal/acoustic/gas publishers
        and mesh impairment relays.
    """
    rgb_config = (
        Path(__file__).resolve().parents[1]
        / "src"
        / "aeris_perception"
        / "config"
        / "perception_demo_rgb_ingest.yaml"
    )
    rgb_source_type = LaunchConfiguration("rgb_source_type")
    rgb_source_uri = LaunchConfiguration("rgb_source_uri")
    rgb_source_name = LaunchConfiguration("rgb_source_name")
    rgb_loop_replay = LaunchConfiguration("rgb_loop_replay")

    nodes = [
        DeclareLaunchArgument(
            "rgb_source_type",
            default_value="camera",
            description="RGB ingest source type: camera, video_file, or replay.",
        ),
        DeclareLaunchArgument(
            "rgb_source_uri",
            default_value="0",
            description="RGB ingest camera index or local file path.",
        ),
        DeclareLaunchArgument(
            "rgb_source_name",
            default_value="halo_front_camera",
            description="Human-readable RGB source name for metadata.",
        ),
        DeclareLaunchArgument(
            "rgb_loop_replay",
            default_value="false",
            description="Whether replay inputs should loop when they reach EOF.",
        ),
        Node(
            package="aeris_perception",
            executable="rgb_ingest",
            name="rgb_ingest",
            output="screen",
            parameters=[
                str(rgb_config),
                {
                    "source_type": rgb_source_type,
                    "source_uri": rgb_source_uri,
                    "source_name": rgb_source_name,
                    "loop_replay": rgb_loop_replay,
                },
            ],
        ),
        Node(
            package="aeris_perception",
            executable="thermal_hotspot",
            name="thermal_hotspot",
            output="screen",
            parameters=[{"rate_hz": 5.0}],
        ),
        Node(
            package="aeris_perception",
            executable="acoustic_bearing",
            name="acoustic_bearing",
            output="screen",
            parameters=[
                {
                    "audio_topic": "acoustic/audio",
                    "publish_topic": "acoustic/bearing",
                    "rate_hz": 1.0,
                    "window_size": 2048,
                    "source_separation_deg": 25.0,
                }
            ],
        ),
        Node(
            package="aeris_perception",
            executable="acoustic_audio_sim",
            name="acoustic_audio_sim",
            output="screen",
            parameters=[
                {
                    "output_topic": "acoustic/audio",
                    "publish_rate_hz": 5.0,
                    "window_size": 2048,
                }
            ],
        ),
        Node(
            package="aeris_perception",
            executable="gas_input_sim",
            name="gas_input_sim",
            output="screen",
            parameters=[
                {
                    "gas_output_topic": "sim/gas/sample",
                    "wind_output_topic": "sim/wind/vector",
                    "sample_rate_hz": 4.0,
                    "wind_rate_hz": 2.0,
                    "source_x": 0.0,
                    "source_y": 0.0,
                    "base_concentration": 3.0,
                    "frame_id": "map",
                }
            ],
        ),
        Node(
            package="aeris_perception",
            executable="gas_isopleth",
            name="gas_isopleth",
            output="screen",
            parameters=[
                {
                    "rate_hz": 0.8,
                    "gas_input_topic": "sim/gas/sample",
                    "wind_input_topic": "sim/wind/vector",
                    "output_topic": "gas/isopleth",
                    "smoothing_window": 30,
                    "plume_resolution": 24,
                    "expected_frame_id": "map",
                }
            ],
        ),
        Node(
            package="aeris_mesh_agent",
            executable="impairment_relay",
            name="impairment_relay",
            output="screen",
            parameters=[
                {
                    "input_topic": "orchestrator/heartbeat",
                    "output_topic": "mesh/heartbeat_imp",
                    "drop_prob": 0.01,
                    "delay_ms": 10,
                }
            ],
        ),
        Node(
            package="aeris_mesh_agent",
            executable="store_forward_tiles",
            name="store_forward_tiles",
            output="screen",
            parameters=[
                {
                    "input_topic": "map/tiles",
                    "output_topic": "map/tiles_out",
                    "link_up": True,
                }
            ],
        ),
        LogInfo(
            msg=(
                "Perception demo running with RGB ingest, thermal/acoustic/gas "
                "pipeline, and mesh impairments. Verify RGB frames with `ros2 topic "
                "hz rgb/image_raw`, acoustic output cadence with `ros2 topic hz "
                "acoustic/bearing`, gas cadence with `ros2 topic hz gas/isopleth`, "
                "and override the RGB path with launch args such as "
                "`rgb_source_type:=replay rgb_source_uri:=/absolute/path/capture.mp4`."
            )
        ),
    ]

    return LaunchDescription(nodes)
