"""Launch file for replaying recorded rosbag sessions in simulation.

This launch file orchestrates the replay of recorded ROS 2 bag files within
a Gazebo simulation environment. It enables mission analysis, debugging, and
visualization of previously recorded flight data by synchronizing world
visualization with rosbag playback.

The launch sequence:
    1. Optionally launches Gazebo with the specified world (headless or GUI).
    2. Configures and executes rosbag play with customizable playback options.

Typical use cases include:
    - Post-flight mission analysis and review
    - Algorithm debugging with recorded sensor data
    - Visualization of historical flight trajectories
    - Regression testing with recorded datasets

Example usage:
    $ ros2 launch aeris_sim replay_recording.launch.py bag:=/path/to/rosbag
    $ ros2 launch aeris_sim replay_recording.launch.py bag:=/path/to/rosbag speed:=2.0
    $ ros2 launch aeris_sim replay_recording.launch.py bag:=/path/to/rosbag loop:=true

Attributes:
    TOOLS_DIR (Path): Absolute path to the simulation tools directory.
    RUN_SIM (Path): Absolute path to the basic simulation runner script.
    DEFAULT_WORLD (str): Default world file path for visualization.
"""

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Path constants relative to this launch file location
TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
RUN_SIM = TOOLS_DIR / "run_basic_sim.sh"
DEFAULT_WORLD = "software/sim/worlds/disaster_scene.sdf"


def _bool(value: str) -> bool:
    """Convert string value to boolean.

    Interprets common string representations of boolean values including
    'true', '1', 'yes', and 'on' as True (case-insensitive).

    Args:
        value: String value to convert.

    Returns:
        True if the value represents a boolean True, False otherwise.
    """
    return value.lower() in {"true", "1", "yes", "on"}


def build_processes(context):
    """Build list of processes for launch execution.

    Resolves launch configurations and constructs the appropriate list of
    ExecuteProcess actions for Gazebo world launch and rosbag playback.

    Args:
        context: LaunchContext providing access to launch configuration values.

    Returns:
        List of ExecuteProcess actions for Gazebo and rosbag playback.

    Raises:
        RuntimeError: If the 'bag' argument is not provided or if the
            specified world file cannot be found.
    """
    # Resolve required launch arguments
    bag = LaunchConfiguration("bag").perform(context)
    if not bag:
        raise RuntimeError("bag launch argument is required")

    # Resolve optional playback configuration arguments
    speed = LaunchConfiguration("speed").perform(context)
    loop = _bool(LaunchConfiguration("loop").perform(context))
    auto_world = _bool(LaunchConfiguration("auto_launch_world").perform(context))
    world_path = LaunchConfiguration("world_path").perform(context)
    headless = _bool(LaunchConfiguration("headless").perform(context))

    # Parse extra arguments for rosbag play command
    extra_args = [arg for arg in LaunchConfiguration("extra_args").perform(context).split() if arg]

    actions = []

    # Conditionally launch Gazebo world for visualization context
    if auto_world:
        world_file = Path(world_path)

        # Resolve relative world paths to absolute paths
        if not world_file.is_absolute():
            world_file = (Path(__file__).resolve().parents[2] / world_file).resolve()

        # Validate world file existence
        if not world_file.is_file():
            raise RuntimeError(f"World file not found: {world_file}")

        # Configure environment for simulation process
        env = os.environ.copy()
        env["WORLD_PATH"] = str(world_file)
        if headless:
            env["HEADLESS"] = "1"

        # Add Gazebo launch process
        actions.append(
            ExecuteProcess(
                cmd=[str(RUN_SIM)],
                output="screen",
                additional_env=env,
            )
        )

    # Construct rosbag play command with optional arguments
    cmd = ["ros2", "bag", "play", bag]

    # Add playback rate multiplier if specified
    if speed:
        cmd += ["-r", speed]

    # Add loop flag if enabled
    if loop:
        cmd.append("-l")

    # Append any extra user-provided arguments
    cmd += extra_args

    # Add rosbag playback process
    actions.append(
        ExecuteProcess(
            cmd=cmd,
            output="screen",
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for rosbag replay with optional world.

    Declares all configurable launch arguments for replaying recorded sessions
    and sets up the OpaqueFunction to build processes at launch time.

    Launch Arguments:
        bag (str, required): Path to the rosbag directory to replay.
        speed (float): Playback speed multiplier (e.g., '2.0' for 2x speed).
            Default: '1.0'
        loop (bool): Whether to loop the rosbag playback continuously.
            Default: 'false'
        extra_args (str): Additional arguments to pass to 'ros2 bag play'.
            Default: ''
        auto_launch_world (bool): Whether to launch Gazebo world for context.
            Default: 'true'
        world_path (str): Path to the Gazebo world file for visualization.
            Default: 'software/sim/worlds/disaster_scene.sdf'
        headless (bool): Whether to run Gazebo in headless mode (no GUI).
            Default: 'true'

    Returns:
        LaunchDescription containing all launch arguments and the process builder.
    """
    return LaunchDescription(
        [
            # Required: Path to rosbag directory
            DeclareLaunchArgument(
                "bag",
                description="Path to rosbag directory to replay"
            ),
            # Playback speed configuration
            DeclareLaunchArgument(
                "speed",
                default_value="1.0",
                description="Playback speed multiplier (1.0 = normal speed)"
            ),
            # Loop configuration
            DeclareLaunchArgument(
                "loop",
                default_value="false",
                description="Loop rosbag playback continuously"
            ),
            # Extra arguments passthrough
            DeclareLaunchArgument(
                "extra_args",
                default_value="",
                description="Additional arguments passed to 'ros2 bag play'"
            ),
            # World auto-launch control
            DeclareLaunchArgument(
                "auto_launch_world",
                default_value="true",
                description="Launch Gazebo world for visualization context"
            ),
            # World file path
            DeclareLaunchArgument(
                "world_path",
                default_value=DEFAULT_WORLD,
                description="Path to Gazebo world file for visualization"
            ),
            # Headless mode control
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run Gazebo in headless mode (no GUI)"
            ),
            # Opaque function to resolve configurations at launch time
            OpaqueFunction(function=build_processes),
        ]
    )
