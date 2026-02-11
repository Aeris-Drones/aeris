"""Launch file for basic Gazebo world scaffold.

This launch file provides a minimal template for launching Gazebo simulation
environments. It serves as a foundation for more complex simulation configurations
and demonstrates the basic structure of a ROS 2 launch file for Gazebo integration.

This is a minimal scaffold launch file intended to be extended when integrating
Gazebo Sim (gz sim) with ros_gz_bridge for specific world configurations and
sensor setups.

Typical use cases include:
    - Template for new simulation environment launch files
    - Minimal Gazebo world loading without additional nodes
    - Base configuration for iterative simulation development

Example usage:
    $ ros2 launch aeris_sim sim_world.launch.py
    $ ros2 launch aeris_sim sim_world.launch.py world:=/path/to/custom_world.sdf

Note:
    This launch file only declares the world argument and does not launch
    Gazebo itself. Extend this file with ExecuteProcess actions or include
    it in other launch files with additional simulation components.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for basic world scaffold.

    Creates a minimal launch description that declares the world file path
    argument. This function serves as a template and foundation for more
    complex simulation launch configurations.

    Launch Arguments:
        world (str): Path to the Gazebo world file in SDF format.
            Default: 'software/sim/worlds/basic_world.sdf'

    Returns:
        LaunchDescription containing the world path declaration.

    Example:
        To extend this launch file with Gazebo execution::

            gazebo = ExecuteProcess(
                cmd=["gz", "sim", LaunchConfiguration("world")],
                output="screen",
            )
            return LaunchDescription([
                DeclareLaunchArgument(...),
                gazebo,
            ])
    """
    # Resolve world path from launch configuration
    world = LaunchConfiguration("world")

    return LaunchDescription([
        # Declare the world file path argument
        # This allows users to specify custom world files at launch time
        DeclareLaunchArgument(
            "world",
            default_value="software/sim/worlds/basic_world.sdf",
            description="Path to the Gazebo world file (SDF format) for basic simulation",
        ),
    ])
