from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution as PathJoin
import os

# Main Launch for MKZ DBW Control, allows for path collection and following

def generate_launch_description():
    vehicle_control_mkz_share_dir = get_package_share_directory('vehicle_control_mkz')

    return LaunchDescription([
        DeclareLaunchArgument('SIM',            default_value='True',           description='Real or Simulated Odometry'),
        DeclareLaunchArgument('IS_CP',          default_value='False',          description='Collect waypoints or follow them'),
        DeclareLaunchArgument('VEHICLE_FILE',   default_value='mkz.yaml',       description='Vehicle type, MKZ'),
        DeclareLaunchArgument('WAYPOINTS_FILE', default_value='waypoints.dat',  description='Waypoints file path for following or collecting'),
        DeclareLaunchArgument('DESIRED_SPEED',  default_value='3.0',            description='Desired forward speed when following waypoints'),
        DeclareLaunchArgument('USE_RVIZ',       default_value='False',           description='Use RVIZ for visualization'),

        # Odometry publishing node, options for real and sim. Additional vehicle path RVIZ display.
        Node(
            package = 'vehicle_control_mkz',
            executable = 'odom_pub',
            name = 'odom_publisher_node',
            parameters = [{'SIM': LC('SIM')}],
        ),
        
        # RVIZ node for visualization, only if enabled.
        Node(
            package='rviz2',
            executable='rviz2',
            name='mkz_rviz_node',
            arguments=['-d', os.path.join(vehicle_control_mkz_share_dir, 'mkz.rviz')],
            respawn=True,
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LC('USE_RVIZ')),
        ),
            
        # Collection of nodes when following waypoints
        GroupAction(
            condition=UnlessCondition(LC('IS_CP')),
            actions=[
                # Longitudinal controller, contains PID controller mapping a desired velocity to throttle and brake inputs
                Node(
                    package='vehicle_control_mkz',
                    executable='long_control',
                    name='long_control_node',
                    output='screen',
                    parameters=[
                        {'sim': LC('SIM')},
                        {'vehicle_yaml_path': PathJoin([vehicle_control_mkz_share_dir, LC('VEHICLE_FILE')])},
                    ]
                ),
                # Lateral controller, uses pure pursuit to map a path to steering wheel commands
                Node(
                    package='vehicle_control_mkz',
                    executable='lat_control',
                    name='lat_control_node',
                    output='screen',
                    parameters=[
                        {'sim': LC('SIM')},
                        {'vehicle_yaml_path': PathJoin([vehicle_control_mkz_share_dir, LC('VEHICLE_FILE')])},
                        {'waypoints_file_path': PathJoin([vehicle_control_mkz_share_dir, LC('WAYPOINTS_FILE')])},
                        {'desired_speed': LC('DESIRED_SPEED')},
                    ]
                ),
            ]
        ),
        # Collection of nodes when collecting waypoints
        GroupAction(
            condition=IfCondition(LC('IS_CP')),
            actions=[
                # Waypoint collection node, records waypoints from the vehicle's odometry and writes to a file for later following
                Node(
                    package='vehicle_control_mkz',
                    executable='waypoint_recorder',
                    name='waypoint_collection_node',
                    output='screen',
                    parameters=[
                        {'waypoints_file_path': PathJoin([vehicle_control_mkz_share_dir, LC('WAYPOINTS_FILE')])},
                    ]
                ),
                # Longitudinal controller in forward steering mode, can be used with keyboard teleop to control the vehicle manually
                Node(
                    package='vehicle_control_mkz',
                    executable='long_control',
                    name='long_control_node',
                    output='screen',
                    parameters=[
                        {'sim': LC('SIM')},
                        {'vehicle_yaml_path': PathJoin([vehicle_control_mkz_share_dir, LC('VEHICLE_FILE')])},
                        {'forward_steering': True},
                    ]
                ),
            ]
        ),
    ])
