from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


VALID_POSITION_SOURCE_MODES = {"telemetry_geodetic", "vio_odometry", "auto"}


def launch_setup(context):
    world = LaunchConfiguration('world').perform(context)
    config = LaunchConfiguration('vehicles_config').perform(context)
    scout_model_name = LaunchConfiguration('scout_model_name').perform(context)
    gps_denied_mode = LaunchConfiguration('gps_denied_mode').perform(context)
    position_source_mode = LaunchConfiguration('position_source_mode').perform(context)
    normalized_position_source_mode = (
        position_source_mode.strip().lower().replace('-', '_')
    )
    if normalized_position_source_mode not in VALID_POSITION_SOURCE_MODES:
        supported_modes = ", ".join(sorted(VALID_POSITION_SOURCE_MODES))
        raise RuntimeError(
            f"Invalid position_source_mode '{position_source_mode}'. "
            f"Expected one of: {supported_modes}"
        )
    position_source_mode = normalized_position_source_mode
    launch_orchestrator = LaunchConfiguration('launch_orchestrator').perform(context)
    vio_odom_stale_sec = LaunchConfiguration('vio_odom_stale_sec').perform(context)
    gps_denied_enabled = gps_denied_mode.strip().lower() in {'1', 'true', 'yes', 'on'}
    launch_orchestrator_enabled = launch_orchestrator.strip().lower() in {
        '1',
        'true',
        'yes',
        'on',
    }

    sitl_command = (
        f'./software/sim/tools/run_multi_drone_sitl.py --config {config} --execute'
    )
    if gps_denied_enabled:
        sitl_command += ' --gps-denied --external-vision'

    actions = [
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

    if launch_orchestrator_enabled:
        actions.append(
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-lc',
                    (
                        'ros2 run aeris_orchestrator mission --ros-args '
                        f'-p gps_denied_mode:={gps_denied_mode} '
                        f'-p navigation_position_source:={position_source_mode} '
                        f'-p vio_odom_stale_sec:={vio_odom_stale_sec}'
                    ),
                ],
                output='screen',
            )
        )

    return actions


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
        DeclareLaunchArgument(
            'launch_orchestrator',
            default_value='true',
            description='Launch aeris_orchestrator mission node with selected position-source parameters',
        ),
        DeclareLaunchArgument(
            'vio_odom_stale_sec',
            default_value='1.0',
            description='Maximum age (seconds) for VIO odometry samples before treated as stale',
        ),
        OpaqueFunction(function=launch_setup),
    ])
