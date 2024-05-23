from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration as LC
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('INFRA',            default_value='True',           description='Infrastructure or Vehicle'),
        
        GroupAction(
            condition=IfCondition(LC('INFRA')),
            actions=[
                 Node(
                 package='infraware_ros',
                 executable='SensorStacker',
                 name='infraware_subscriber',
                 parameters = [{'infrastructure':LC('INFRA')}],
                 ),
                 
                 Node(
                 package='infraware_ros',
                 executable='FilterInfra',
                 name='infraware_filter'
                 ),
            ]
        ),
        
        GroupAction(
            condition=UnlessCondition(LC('INFRA')),
            actions=[
                 Node(
                 package='infraware_ros',
                 executable='SensorStacker',
                 name='infraware_subscriber',
                 parameters = [{'infrastructure':LC('INFRA')}],
                 ),
                 
                 Node(
                 package='infraware_ros',
                 executable='FilterVehicle',
                 name='infraware_filter_vehicle'
                 ),
                 Node(
                 package='infraware_ros',
                 executable='FilterVehicleKF',
                 name='infraware_filter_vehicle_kf'
                 ),
                 Node(
                 package='infraware_ros',
                 executable='FilterVehicleCI',
                 name='infraware_filter_vehicle_ci'
                 ),
                 Node(
                 package='infraware_ros',
                 executable='FilterVehicleKFNO',
                 name='infraware_filter_vehicle_kfno'
                 ),
                 
            ]
        ),
        
        
    ])
