#!/usr/bin/env python3
"""Launch Gazebo (Ignition) with the Aeris block world and stub ros_gz_bridge."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    this_dir = Path(__file__).resolve().parent
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=str(this_dir.parent / "worlds" / "aeris_block.world"),
        description="Path to the Gazebo world file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Set to true if downstream nodes should use Gazebo clock",
    )

    world_path = LaunchConfiguration("world")

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-r", world_path],
        output="screen",
        additional_env={"QT_QPA_PLATFORM": "offscreen"},
    )

    bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
        ],
        output="screen",
    )

    logging = LogInfo(
        msg=(
            "Launched Ignition Gazebo Fortress via 'ign gazebo'. "
            "Bridge mapping /camera/image between Gazebo Transport and ROS 2. "
            "Use --ros-args -p use_sim_time:=true on ROS nodes if sim clock is required."
        )
    )

    return LaunchDescription(
        [
            world_arg,
            use_sim_time_arg,
            gazebo,
            bridge,
            logging,
        ]
    )
