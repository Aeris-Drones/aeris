from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world = LaunchConfiguration('world')
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='software/sim/worlds/disaster_scene.sdf',
            description='Path to the disaster scene world.'
        ),
        # Full Gazebo/bridge wiring stays in container launch scripts
    ])

