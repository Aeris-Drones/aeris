from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    world = LaunchConfiguration('world').perform(context)
    config = LaunchConfiguration('vehicles_config').perform(context)
    scout_model_name = LaunchConfiguration('scout_model_name').perform(context)
    gps_denied_mode = LaunchConfiguration('gps_denied_mode').perform(context)
    position_source_mode = LaunchConfiguration('position_source_mode').perform(context)
    gps_denied_enabled = gps_denied_mode.strip().lower() in {'1', 'true', 'yes', 'on'}

    sitl_command = (
        f'./software/sim/tools/run_multi_drone_sitl.py --config {config} --execute'
    )
    if gps_denied_enabled:
        sitl_command += ' --gps-denied --external-vision'

    return [
        ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                f'WORLD_PATH={world} SCOUT_MODEL_NAME={scout_model_name} '
                f'GPS_DENIED_MODE={gps_denied_mode} '
                f'POSITION_SOURCE_MODE={position_source_mode} '
                f'./software/sim/tools/run_basic_sim.sh',
            ],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['bash', '-lc', sitl_command],
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
        DeclareLaunchArgument(
            'scout_model_name',
            default_value='scout1',
            description='Scout model name used for stereo/IMU bridge remaps',
        ),
        DeclareLaunchArgument(
            'gps_denied_mode',
            default_value='false',
            description='Enable GPS-denied SITL profile with external vision aiding',
        ),
        DeclareLaunchArgument(
            'position_source_mode',
            default_value='telemetry_geodetic',
            description='Mission position source mode (telemetry_geodetic, vio_odometry, auto)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
