from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_root = Path(__file__).resolve().parents[1]

    openvins_config_default = str(package_root / 'config' / 'openvins_sim.yaml')
    rtabmap_config_default = str(package_root / 'config' / 'rtabmap_vio_sim.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    left_image_topic = LaunchConfiguration('left_image_topic')
    right_image_topic = LaunchConfiguration('right_image_topic')
    left_camera_info_topic = LaunchConfiguration('left_camera_info_topic')
    right_camera_info_topic = LaunchConfiguration('right_camera_info_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    openvins_odom_topic = LaunchConfiguration('openvins_odom_topic')
    rtabmap_database_path = LaunchConfiguration('rtabmap_database_path')
    openvins_log_directory = LaunchConfiguration('openvins_log_directory')

    openvins_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='openvins',
        output='screen',
        parameters=[
            LaunchConfiguration('openvins_config'),
            {
                'use_sim_time': use_sim_time,
                'topic_left': left_image_topic,
                'topic_right': right_image_topic,
                'topic_left_camera_info': left_camera_info_topic,
                'topic_right_camera_info': right_camera_info_topic,
                'topic_imu': imu_topic,
                'output_odom_topic': openvins_odom_topic,
                'log_directory': openvins_log_directory,
            },
        ],
        remappings=[
            ('/ov_msckf/odomimu', openvins_odom_topic),
        ],
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            LaunchConfiguration('rtabmap_config'),
            {
                'use_sim_time': use_sim_time,
                'frame_id': LaunchConfiguration('base_frame'),
                'odom_frame_id': LaunchConfiguration('odom_frame'),
                'map_frame_id': LaunchConfiguration('map_frame'),
                'Rtabmap/DatabasePath': rtabmap_database_path,
            },
        ],
        remappings=[
            ('left/image_rect', left_image_topic),
            ('right/image_rect', right_image_topic),
            ('left/camera_info', left_camera_info_topic),
            ('right/camera_info', right_camera_info_topic),
            ('imu', imu_topic),
            ('odom', openvins_odom_topic),
            ('grid_map', LaunchConfiguration('occupancy_topic')),
            ('cloud_map', LaunchConfiguration('point_cloud_topic')),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('openvins_config', default_value=openvins_config_default),
        DeclareLaunchArgument('rtabmap_config', default_value=rtabmap_config_default),
        DeclareLaunchArgument('scout_model_name', default_value='scout1'),
        DeclareLaunchArgument(
            'left_image_topic',
            default_value=['/', LaunchConfiguration('scout_model_name'), '/stereo/left/image_raw'],
        ),
        DeclareLaunchArgument(
            'right_image_topic',
            default_value=['/', LaunchConfiguration('scout_model_name'), '/stereo/right/image_raw'],
        ),
        DeclareLaunchArgument(
            'left_camera_info_topic',
            default_value=['/', LaunchConfiguration('scout_model_name'), '/stereo/left/camera_info'],
        ),
        DeclareLaunchArgument(
            'right_camera_info_topic',
            default_value=['/', LaunchConfiguration('scout_model_name'), '/stereo/right/camera_info'],
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value=['/', LaunchConfiguration('scout_model_name'), '/imu/data'],
        ),
        DeclareLaunchArgument(
            'openvins_odom_topic',
            default_value=['/', LaunchConfiguration('scout_model_name'), '/openvins/odom'],
        ),
        DeclareLaunchArgument(
            'openvins_log_directory',
            default_value=['/tmp/aeris/openvins/', LaunchConfiguration('scout_model_name')],
        ),
        DeclareLaunchArgument(
            'rtabmap_database_path',
            default_value=['/tmp/aeris/rtabmap/', LaunchConfiguration('scout_model_name'), '.db'],
        ),
        DeclareLaunchArgument('occupancy_topic', default_value='/map'),
        DeclareLaunchArgument('point_cloud_topic', default_value='/rtabmap/cloud_map'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        openvins_node,
        rtabmap_node,
    ])
