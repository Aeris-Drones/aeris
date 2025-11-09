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

    world_path = LaunchConfiguration("world")

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", world_path],
        output="screen",
    )

    bridge_stub = LogInfo(
        msg=(
            "ros_gz_bridge TODO: add camera/IMU/topic remaps after Phase 1 "
            "sensors are finalized"
        )
    )

    return LaunchDescription(
        [
            world_arg,
            gazebo,
            bridge_stub,
        ]
    )
