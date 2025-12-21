"""
Nav2 Costmap検証用launchファイル
- ロボットは動かない
- コストマップとコーン検出のみ起動
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mirs_pkg = get_package_share_directory('mirs')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    cone_detector_pkg = get_package_share_directory('cone_detector')

    nav2_params_file = os.path.join(mirs_pkg, 'config', 'nav2_params.yaml')

    # --- 1. LiDAR + TF (ハードウェア) ---
    # mirs.launch.pyからLiDARとTFを起動
    mirs_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mirs_pkg, 'launch', 'mirs.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )

    # --- 2. Nav2 (コストマップのみ) ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
        }.items()
    )

    # --- 3. ScanToPointCloud + コーンクラスタリング ---
    scan_to_pointcloud_node = Node(
        package='cone_detector',
        executable='scan_to_pointcloud',
        name='scan_to_pointcloud',
        output='screen'
    )

    cone_cluster_node = Node(
        package='cone_detector',
        executable='cone_cluster_node',
        name='cone_cluster_node',
        output='screen'
    )

    # --- 4. ConeAreaNode (/accumulated_cones を出力) ---
    cone_area_node = Node(
        package='cone_detector',
        executable='cone_area_node',
        name='cone_area_node',
        output='screen',
        remappings=[
            ('/confirmed_cones', '/cone_centers')  # LiDARのみモード
        ]
    )

    # --- 5. RViz2 ---
    rviz_config_file = os.path.join(mirs_pkg, 'rviz', 'system_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # コーン検出を少し遅延させてTFが準備できるのを待つ
    delayed_cone_nodes = TimerAction(
        period=3.0,
        actions=[
            scan_to_pointcloud_node,
            cone_cluster_node,
            cone_area_node
        ]
    )

    return LaunchDescription([
        mirs_hardware_launch,
        nav2_launch,
        delayed_cone_nodes,
        rviz_node
    ])
