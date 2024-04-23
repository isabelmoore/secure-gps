from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration as LC
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('Spoofed',            default_value='False',           description='Spoofed GPS or not'),
                GroupAction(
                    condition=IfCondition(LC('Spoofed')),
                    actions=[
                        Node(
                        package='infraware_ros',
                        executable='GPSSpoof',
                        name='spoof'
                        ),
                    ]
                ),

                Node(
                package='infraware_ros',
                executable='Cartesian',
                name='cartesian',
                ),
        
    ])
