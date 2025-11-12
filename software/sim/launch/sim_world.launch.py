from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world = LaunchConfiguration('world')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='software/sim/worlds/basic_world.sdf'),
        # Placeholder: integrate gz sim + bridge here per environment
        # This file serves as a scaffold; implementation will be completed once dependencies are verified.
    ])

