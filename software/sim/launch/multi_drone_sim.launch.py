from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    world = LaunchConfiguration('world').perform(context)
    config = LaunchConfiguration('vehicles_config').perform(context)

    return [
        ExecuteProcess(
            cmd=['bash', '-lc', f'WORLD_PATH={world} ./software/sim/tools/run_basic_sim.sh'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['bash', '-lc', f'./software/sim/tools/run_multi_drone_sitl.py --config {config} --execute'],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='software/sim/worlds/disaster_scene.sdf',
            description='World file to load before spawning multiple drones',
        ),
        DeclareLaunchArgument(
            'vehicles_config',
            default_value='software/sim/config/multi_drone.yaml',
            description='Vehicle config describing poses and MAVLink ports',
        ),
        OpaqueFunction(function=launch_setup),
    ])

