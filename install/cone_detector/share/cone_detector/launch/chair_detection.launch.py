import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    scan_to_pointcloud = Node(
        package='chair_detector',  # scan_to_pointcloudが含まれるパッケージ名
        executable='scan_to_pointcloud',  # ノードの実行ファイル名
        name='scan_to_pointcloud',
        output='screen',  # ログ出力をスクリーンに表示
        #parameters=[]
    )

    leg_detecor = Node(
        package='chair_detector',  # leg_detectorが含まれるパッケージ名
        executable='leg_detector',  # ノードの実行ファイル名
        name='leg_detector',
        output='screen',  # ログ出力をスクリーンに表示
        #parameters=[]
    )

    chair_detecor = Node(
        package='chair_detector',  # chair_detectorが含まれるパッケージ名
        executable='chair_detector',  # ノードの実行ファイル名
        name='chair_detector',
        output='screen',  # ログ出力をスクリーンに表示
        #parameters=[]
    )

    #laser_filter = Node(
    #    package   = 'laser_filters',
    #    executable= 'scan_to_scan_filter_chain',
    #    parameters=['/home/oumuika/Documents/mirs2403/src/chair_detector/config/range_filter.yaml'],
    #)

    ld = LaunchDescription()
    ld.add_action(scan_to_pointcloud)
    ld.add_action(leg_detecor)
    ld.add_action(chair_detecor)
    #ld.add_action(laser_filter)

    return ld
