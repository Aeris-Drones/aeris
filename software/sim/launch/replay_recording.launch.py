from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
RUN_SIM = TOOLS_DIR / "run_basic_sim.sh"
DEFAULT_WORLD = "software/sim/worlds/disaster_scene.sdf"


def _bool(value: str) -> bool:
    return value.lower() in {"true", "1", "yes", "on"}


def build_processes(context):
    bag = LaunchConfiguration("bag").perform(context)
    if not bag:
        raise RuntimeError("bag launch argument is required")
    speed = LaunchConfiguration("speed").perform(context)
    loop = _bool(LaunchConfiguration("loop").perform(context))
    auto_world = _bool(LaunchConfiguration("auto_launch_world").perform(context))
    world_path = LaunchConfiguration("world_path").perform(context)
    headless = _bool(LaunchConfiguration("headless").perform(context))
    extra_args = [arg for arg in LaunchConfiguration("extra_args").perform(context).split() if arg]

    actions = []
    if auto_world:
        world_file = Path(world_path)
        if not world_file.is_absolute():
            world_file = (Path(__file__).resolve().parents[2] / world_file).resolve()
        if not world_file.is_file():
            raise RuntimeError(f"World file not found: {world_file}")
        env = os.environ.copy()
        env["WORLD_PATH"] = str(world_file)
        if headless:
            env["HEADLESS"] = "1"
        actions.append(
            ExecuteProcess(
                cmd=[str(RUN_SIM)],
                output="screen",
                additional_env=env,
            )
        )

    cmd = ["ros2", "bag", "play", bag]
    if speed:
        cmd += ["-r", speed]
    if loop:
        cmd.append("-l")
    cmd += extra_args

    actions.append(
        ExecuteProcess(
            cmd=cmd,
            output="screen",
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", description="Path to rosbag directory"),
            DeclareLaunchArgument("speed", default_value="1.0"),
            DeclareLaunchArgument("loop", default_value="false"),
            DeclareLaunchArgument("extra_args", default_value=""),
            DeclareLaunchArgument("auto_launch_world", default_value="true"),
            DeclareLaunchArgument("world_path", default_value=DEFAULT_WORLD),
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=build_processes),
        ]
    )
