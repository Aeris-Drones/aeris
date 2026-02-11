"""Launch file for perception pipeline demonstration.

Spawns synthetic sensor publishers (thermal, acoustic, gas) and mesh network
impairment shims to simulate degraded communication conditions for bench testing.
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the perception demo.

    Returns:
        LaunchDescription: A launch description containing all nodes for the
        perception demonstration, including thermal/acoustic/gas publishers
        and mesh impairment relays.
    """
    nodes = [
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
            parameters=[{"rate_hz": 1.0}],
        ),
        Node(
            package="aeris_perception",
            executable="gas_isopleth",
            name="gas_isopleth",
            output="screen",
            parameters=[{"rate_hz": 0.5}],
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
                "Perception demo running — thermal/acoustic/gas publishers plus mesh "
                "impairments. Use 'ros2 topic hz' to verify rates and 'ros2 param set "
                "/store_forward_tiles link_up false|true' to toggle buffering."
            )
        ),
    ]

    return LaunchDescription(nodes)
