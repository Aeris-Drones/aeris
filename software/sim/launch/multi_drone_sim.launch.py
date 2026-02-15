"""Launch file for multi-drone simulation with SITL and orchestration.

This launch file orchestrates a complete multi-drone simulation environment,
integrating Gazebo/Ignition simulator, Software-In-The-Loop (SITL) instances
for each drone, and the Aeris mission orchestrator. It supports various
position source modes including GPS telemetry, Visual-Inertial Odometry (VIO),
and automatic selection.

The launch sequence:
    1. Launches Gazebo with the specified world file.
    2. Spawns multiple SITL instances based on vehicle configuration.
    3. Optionally launches the mission orchestrator node.

Typical use cases include:
    - Multi-UAV swarm coordination testing
    - GPS-denied navigation algorithm validation
    - Mission planning and execution simulation
    - Communication and coordination protocol testing

Example usage:
    $ ros2 launch aeris_sim multi_drone_sim.launch.py
    $ ros2 launch aeris_sim multi_drone_sim.launch.py gps_denied_mode:=true
    $ ros2 launch aeris_sim multi_drone_sim.launch.py position_source_mode:=vio_odometry

Attributes:
    VALID_POSITION_SOURCE_MODES (set): Supported position source configurations.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Valid position source modes for mission navigation
VALID_POSITION_SOURCE_MODES = {"telemetry_geodetic", "vio_odometry", "auto"}


def launch_setup(context):
    """Configure and build launch actions based on context.

    Resolves launch configurations, validates position source mode, and constructs
    the list of processes to execute including Gazebo, SITL instances, and
    optionally the mission orchestrator.

    Args:
        context: LaunchContext providing access to launch configuration values.

    Returns:
        List of ExecuteProcess actions to be launched.

    Raises:
        RuntimeError: If an invalid position_source_mode is specified.
    """
    # Resolve launch configuration values from context
    world = LaunchConfiguration('world').perform(context)
    config = LaunchConfiguration('vehicles_config').perform(context)
    scout_model_name = LaunchConfiguration('scout_model_name').perform(context)
    gps_denied_mode = LaunchConfiguration('gps_denied_mode').perform(context)
    position_source_mode = LaunchConfiguration('position_source_mode').perform(context)

    # Normalize position source mode for case-insensitive comparison
    normalized_position_source_mode = (
        position_source_mode.strip().lower().replace('-', '_')
    )

    # Validate position source mode against supported options
    if normalized_position_source_mode not in VALID_POSITION_SOURCE_MODES:
        supported_modes = ", ".join(sorted(VALID_POSITION_SOURCE_MODES))
        raise RuntimeError(
            f"Invalid position_source_mode '{position_source_mode}'. "
            f"Expected one of: {supported_modes}"
        )
    position_source_mode = normalized_position_source_mode

    # Resolve additional configuration parameters
    launch_orchestrator = LaunchConfiguration('launch_orchestrator').perform(context)
    vio_odom_stale_sec = LaunchConfiguration('vio_odom_stale_sec').perform(context)

    # Parse boolean flags from string configuration values
    gps_denied_enabled = gps_denied_mode.strip().lower() in {'1', 'true', 'yes', 'on'}
    launch_orchestrator_enabled = launch_orchestrator.strip().lower() in {
        '1',
        'true',
        'yes',
        'on',
    }

    # Construct SITL command with optional GPS-denied and external vision flags
    sitl_command = (
        f'./software/sim/tools/run_multi_drone_sitl.py --config {config} --execute'
    )
    if gps_denied_enabled:
        sitl_command += ' --gps-denied --external-vision'

    # Initialize list of processes to launch
    actions = [
        # Launch Gazebo simulator with environment configuration
        ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                f'WORLD_PATH={world} SCOUT_MODEL_NAME={scout_model_name} '
                f'GPS_DENIED_MODE={gps_denied_mode} '
                f'POSITION_SOURCE_MODE={position_source_mode} '
                f'VEHICLES_CONFIG={config} '
                f'./software/sim/tools/run_basic_sim.sh',
            ],
            output='screen',
        ),
        # Launch multi-drone SITL instances
        ExecuteProcess(
            cmd=['bash', '-lc', sitl_command],
            output='screen',
        ),
    ]

    # Conditionally launch mission orchestrator node
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


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for multi-drone simulation.

    Declares all configurable launch arguments for the simulation environment
    and sets up the OpaqueFunction to resolve configurations at launch time.

    Launch Arguments:
        world (str): Path to the Gazebo world file.
            Default: 'software/sim/worlds/disaster_scene.sdf'
        vehicles_config (str): YAML configuration file describing vehicle poses
            and MAVLink communication ports.
            Default: 'software/sim/config/multi_drone.yaml'
        scout_model_name (str): Name of the scout model for stereo camera and
            IMU bridge topic remapping.
            Default: 'scout1'
        gps_denied_mode (str): Enable GPS-denied SITL profile with external
            vision aiding for VIO testing.
            Default: 'false'
        position_source_mode (str): Navigation position source mode selection.
            Options: 'telemetry_geodetic', 'vio_odometry', 'auto'
            Default: 'telemetry_geodetic'
        launch_orchestrator (str): Whether to launch the aeris_orchestrator
            mission node with selected position-source parameters.
            Default: 'true'
        vio_odom_stale_sec (float): Maximum age in seconds for VIO odometry
            samples before they are considered stale and ignored.
            Default: '1.0'

    Returns:
        LaunchDescription containing all launch arguments and the setup function.
    """
    return LaunchDescription([
        # World configuration
        DeclareLaunchArgument(
            'world',
            default_value='software/sim/worlds/disaster_scene.sdf',
            description='World file to load before spawning multiple drones',
        ),
        # Vehicle configuration
        DeclareLaunchArgument(
            'vehicles_config',
            default_value='software/sim/config/multi_drone.yaml',
            description='Vehicle config describing poses and MAVLink ports',
        ),
        # Model naming for topic remapping
        DeclareLaunchArgument(
            'scout_model_name',
            default_value='scout1',
            description='Scout model name used for stereo/IMU bridge remaps',
        ),
        # GPS-denied simulation mode
        DeclareLaunchArgument(
            'gps_denied_mode',
            default_value='false',
            description='Enable GPS-denied SITL profile with external vision aiding',
        ),
        # Position source selection for navigation
        DeclareLaunchArgument(
            'position_source_mode',
            default_value='telemetry_geodetic',
            description='Mission position source mode (telemetry_geodetic, vio_odometry, auto)',
        ),
        # Mission orchestrator control
        DeclareLaunchArgument(
            'launch_orchestrator',
            default_value='true',
            description='Launch aeris_orchestrator mission node with selected position-source parameters',
        ),
        # VIO odometry staleness threshold
        DeclareLaunchArgument(
            'vio_odom_stale_sec',
            default_value='1.0',
            description='Maximum age (seconds) for VIO odometry samples before treated as stale',
        ),
        # Opaque function to resolve configurations at launch time
        OpaqueFunction(function=launch_setup),
    ])
