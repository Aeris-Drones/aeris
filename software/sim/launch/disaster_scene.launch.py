import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _merge_path(new_path: str, env_key: str) -> str:
    existing = os.environ.get(env_key)
    if existing:
        return f"{new_path}:{existing}"
    return new_path


def generate_launch_description():
    world = LaunchConfiguration('world')
    auto_launch = LaunchConfiguration('auto_launch')

    workspace = Path(__file__).resolve().parents[2]
    model_path = str(workspace / 'software' / 'sim' / 'models')
    plugin_path = str(workspace / 'install' / 'aeris_camera_controller' / 'lib')

    ign_resource_path = _merge_path(model_path, 'IGN_GAZEBO_RESOURCE_PATH')
    gz_resource_path = _merge_path(model_path, 'GZ_SIM_RESOURCE_PATH')
    plugin_search_path = _merge_path(plugin_path, 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH')

    ign_launch = ExecuteProcess(
        condition=IfCondition(auto_launch),
        cmd=['ign', 'gazebo', world, '-r', '-s', '-v', '3'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='software/sim/worlds/disaster_scene.sdf',
            description='Path to the disaster scene world.'
        ),
        DeclareLaunchArgument(
            'auto_launch',
            default_value='false',
            description='Set true to spawn Ignition Gazebo directly from this launch file.'
        ),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_search_path),
        ign_launch,
    ])
