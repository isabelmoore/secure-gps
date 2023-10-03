import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    dbw_gazebo_mkz_share_dir = get_package_share_directory('dbw_gazebo_mkz')                # This package
    dataspeed_dbw_gazebo_share_dir = get_package_share_directory('dataspeed_dbw_gazebo')    # Dataspeed's Gazebo package

    return LaunchDescription([
        # These aren't currently in use but are here for reference
        DeclareLaunchArgument('camera_value', default_value='0', description='Camera value'),
        DeclareLaunchArgument('use_vision', default_value='False', description='Use vision'),
        DeclareLaunchArgument('model', default_value='mkz', description='Vehicle model'),

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(dataspeed_dbw_gazebo_share_dir,
                    'launch', 'dataspeed_dbw_gazebo.launch.xml')),
            launch_arguments={
                'use_camera_control': 'true',
                'world_name': os.path.join(dbw_gazebo_mkz_share_dir, 'worlds', 'test_track.world'),
                'sim_param_file': os.path.join(dbw_gazebo_mkz_share_dir, 'yaml', 'single_vehicle_test_track.yaml'),
                'headless': 'false',
                'pause': 'false'
            }.items()
        ),

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(dbw_gazebo_mkz_share_dir,
                        'launch', 'generic_dataspeed_dbw.launch.xml')),
            launch_arguments={
                'vehicle_name': 'vehicle',
                'vehicle_type': LaunchConfiguration('model')
            }.items()
        ),

        Node(
            package='vehicle_control_mkz',
            executable='gazebo_twister',
            name='vehicle_initialize'
        )
    ])
