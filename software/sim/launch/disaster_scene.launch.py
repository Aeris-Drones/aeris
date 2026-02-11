"""Launch file for disaster scenario simulation in Gazebo/Ignition.

This launch file configures and initializes a disaster scene simulation environment
using Gazebo (Ignition) Fortress. It sets up the necessary resource paths for models
and plugins, and optionally auto-launches the simulator with a disaster scene world.

Typical use cases include:
    - Search and rescue mission simulation
    - Multi-drone coordination in disaster zones
    - Perception algorithm testing in realistic debris environments
    - Emergency response scenario training

Example usage:
    $ ros2 launch aeris_sim disaster_scene.launch.py
    $ ros2 launch aeris_sim disaster_scene.launch.py auto_launch:=true

Attributes:
    DEFAULT_WORLD (str): Relative path to the default disaster scene world file.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _merge_path(new_path: str, env_key: str) -> str:
    """Merge a new path with an existing environment variable.

    Prepends the new path to the existing environment variable value,
    using ':' as the separator. If the environment variable does not
    exist, returns only the new path.

    Args:
        new_path: The new directory path to prepend.
        env_key: The name of the environment variable to merge with.

    Returns:
        The merged path string with new_path prepended to existing paths.
    """
    existing = os.environ.get(env_key)
    if existing:
        return f"{new_path}:{existing}"
    return new_path


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for disaster scene simulation.

    Configures the simulation environment by:
        1. Resolving workspace paths for models and plugins.
        2. Setting up Gazebo/Ignition resource environment variables.
        3. Conditionally launching Gazebo with the disaster scene world.

    Launch Arguments:
        world (str): Path to the disaster scene world file.
            Default: 'software/sim/worlds/disaster_scene.sdf'
        auto_launch (bool): If 'true', spawns Ignition Gazebo directly.
            Default: 'false'

    Returns:
        LaunchDescription containing all actions and configurations for the
        disaster scene simulation.
    """
    # Resolve launch configuration substitutions
    world = LaunchConfiguration('world')
    auto_launch = LaunchConfiguration('auto_launch')

    # Determine workspace paths relative to this launch file location
    # Assumes launch file is located at: workspace/software/sim/launch/
    workspace = Path(__file__).resolve().parents[2]
    model_path = str(workspace / 'software' / 'sim' / 'models')
    plugin_path = str(workspace / 'install' / 'aeris_camera_controller' / 'lib')

    # Merge paths with existing environment variables to preserve user configuration
    ign_resource_path = _merge_path(model_path, 'IGN_GAZEBO_RESOURCE_PATH')
    gz_resource_path = _merge_path(model_path, 'GZ_SIM_RESOURCE_PATH')
    plugin_search_path = _merge_path(plugin_path, 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH')

    # Configure Gazebo execution process
    # -r: Run simulation on startup
    # -s: Use simulation time
    # -v 3: Set verbosity level to 3 (info)
    ign_launch = ExecuteProcess(
        condition=IfCondition(auto_launch),
        cmd=['ign', 'gazebo', world, '-r', '-s', '-v', '3'],
        output='screen'
    )

    return LaunchDescription([
        # Declare launch arguments for configurable simulation parameters
        DeclareLaunchArgument(
            'world',
            default_value='software/sim/worlds/disaster_scene.sdf',
            description='Path to the disaster scene world file (SDF format).'
        ),
        DeclareLaunchArgument(
            'auto_launch',
            default_value='false',
            description='Set to "true" to spawn Ignition Gazebo directly from this launch file.'
        ),

        # Set environment variables for Gazebo/Ignition resource discovery
        # IGN_GAZEBO_RESOURCE_PATH: Model and world file search path (Ignition)
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        # GZ_SIM_RESOURCE_PATH: Model and world file search path (Gazebo Sim)
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        # IGN_GAZEBO_SYSTEM_PLUGIN_PATH: Plugin library search path
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_search_path),

        # Conditional Gazebo launch based on auto_launch argument
        ign_launch,
    ])
