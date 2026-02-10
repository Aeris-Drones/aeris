from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_root = Path(__file__).resolve().parents[1]

    openvins_config_default = str(package_root / 'config' / 'openvins_sim.yaml')
    rtabmap_config_default = str(package_root / 'config' / 'rtabmap_vio_sim.yaml')
    map_tile_config_default = str(package_root / 'config' / 'map_tile_stream.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    left_image_topic = LaunchConfiguration('left_image_topic')
    right_image_topic = LaunchConfiguration('right_image_topic')
    left_camera_info_topic = LaunchConfiguration('left_camera_info_topic')
    right_camera_info_topic = LaunchConfiguration('right_camera_info_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    openvins_odom_topic = LaunchConfiguration('openvins_odom_topic')
    map_tile_topic = LaunchConfiguration('map_tile_topic')
    tile_service_name = LaunchConfiguration('tile_service_name')
    mbtiles_path = LaunchConfiguration('mbtiles_path')

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

    map_tile_node = Node(
        package='aeris_map',
        executable='map_tile_publisher',
        name='map_tile_publisher',
        output='screen',
        parameters=[
            LaunchConfiguration('map_tile_config'),
            {
                'use_sim_time': use_sim_time,
                'occupancy_topic': LaunchConfiguration('occupancy_topic'),
                'topic': map_tile_topic,
                'tile_service_name': tile_service_name,
                'mbtiles_path': mbtiles_path,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('openvins_config', default_value=openvins_config_default),
        DeclareLaunchArgument('rtabmap_config', default_value=rtabmap_config_default),
        DeclareLaunchArgument('map_tile_config', default_value=map_tile_config_default),
        DeclareLaunchArgument('left_image_topic', default_value='/scout1/stereo/left/image_raw'),
        DeclareLaunchArgument('right_image_topic', default_value='/scout1/stereo/right/image_raw'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/scout1/stereo/left/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/scout1/stereo/right/camera_info'),
        DeclareLaunchArgument('imu_topic', default_value='/scout1/imu/data'),
        DeclareLaunchArgument('openvins_odom_topic', default_value='/scout1/openvins/odom'),
        DeclareLaunchArgument('occupancy_topic', default_value='/map'),
        DeclareLaunchArgument('point_cloud_topic', default_value='/rtabmap/cloud_map'),
        DeclareLaunchArgument('map_tile_topic', default_value='/map/tiles'),
        DeclareLaunchArgument('tile_service_name', default_value='/map/get_tile_bytes'),
        DeclareLaunchArgument('mbtiles_path', default_value='/tmp/aeris/map_tiles/live_map.mbtiles'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        openvins_node,
        rtabmap_node,
        map_tile_node,
    ])
