#!/usr/bin/env python3
"""Launch file for single block simulation with Gazebo (Ignition) Fortress.

This launch file initializes a minimal simulation environment featuring a
single block world with a basic ROS-Gazebo bridge. It serves as a lightweight
test environment for camera perception, basic navigation, and ROS 2 integration
validation.

The launch sequence:
    1. Configures and launches Ignition Gazebo with the Aeris block world.
    2. Initializes a ros_gz_bridge for camera image topic translation.
    3. Logs informational message about sim time configuration.

Typical use cases include:
    - Camera sensor validation and calibration
    - Basic ROS 2 - Gazebo integration testing
    - Lightweight perception algorithm development
    - Educational demonstrations of robot simulation

Example usage:
    $ ros2 launch aeris_sim sim_block.launch.py
    $ ros2 launch aeris_sim sim_block.launch.py use_sim_time:=true

Note:
    This launch file runs Gazebo in headless mode (offscreen rendering) by
    default. GUI can be enabled by modifying the QT_QPA_PLATFORM environment
    variable or launching Gazebo separately.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for single block simulation.

    Configures the simulation environment by resolving world file paths,
    setting up the Gazebo execution process with headless rendering, and
    initializing the ROS-Gazebo bridge for camera image topics.

    Launch Arguments:
        world (str): Path to the Gazebo world file (SDF or WORLD format).
            Default: '<launch_dir>/../worlds/aeris_block.world'
        use_sim_time (bool): Indicates whether downstream ROS nodes should
            use Gazebo simulation time instead of wall clock time.
            Default: 'false'

    Returns:
        LaunchDescription containing Gazebo process, bridge, and logging actions.
    """
    # Resolve the directory containing this launch file for relative path resolution
    this_dir = Path(__file__).resolve().parent

    # Declare configurable launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=str(this_dir.parent / "worlds" / "aeris_block.world"),
        description="Path to the Gazebo world file (SDF or WORLD format)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Set to 'true' if downstream nodes should use Gazebo simulation clock",
    )

    # Resolve world path from launch configuration
    world_path = LaunchConfiguration("world")

    # Configure Gazebo execution process
    # -s: Use simulation time (step mode)
    # -r: Run simulation on startup
    # QT_QPA_PLATFORM=offscreen: Enable headless rendering (no GUI)
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-r", world_path],
        output="screen",
        additional_env={"QT_QPA_PLATFORM": "offscreen"},
    )

    # Configure ROS-Gazebo bridge for camera image topic
    # Maps /camera/image from Gazebo Transport to ROS 2 sensor_msgs/Image
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

    # Log informational message about simulation configuration
    logging = LogInfo(
        msg=(
            "Launched Ignition Gazebo Fortress via 'ign gazebo'. "
            "Bridge mapping /camera/image between Gazebo Transport and ROS 2. "
            "Use --ros-args -p use_sim_time:=true on ROS nodes if sim clock is required."
        )
    )

    return LaunchDescription(
        [
            # Launch arguments for configuration
            world_arg,
            use_sim_time_arg,

            # Core simulation processes
            gazebo,
            bridge,

            # Informational logging
            logging,
        ]
    )
