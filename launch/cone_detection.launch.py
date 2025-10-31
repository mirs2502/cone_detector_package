import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # カメラ (v4l2_camera) の起動ノード
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        # /dev/video0 はご自身の環境に合わせて変更してください
        parameters=[{'video_device': '/dev/video0'}],
        remappings=[
            ('camera/image_raw', '/image_raw'),
            ('camera/camera_info', '/camera_info')
        ]
    )

    # 静的TF（座標変換）の設定 (base_link -> laser)
    # arguments の値はご自身のロボットの測定値に変更してください
    static_tf_lidar = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        # [X, Y, Z, Yaw, Pitch, Roll] (base_link -> laser)
        arguments = ['0.1', '0.0', '0.2', '0', '0', '0', 'base_link', 'laser']
    )
    
    # 静的TF（座標変換）の設定 (base_link -> camera_link)
    # arguments の値はご自身のロボットの測定値に変更してください
    static_tf_camera = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        # [X, Y, Z, Yaw, Pitch, Roll] (base_link -> camera_link)
        arguments = ['0.15', '0.0', '0.3', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    # Scan -> PointCloud ノード (既存)
    scan_to_pointcloud = Node(
        package='chair_detector',  
        executable='scan_to_pointcloud',  
        name='scan_to_pointcloud',
        output='screen',
    )

    # 【追加】LiDARクラスタリングノード
    cone_cluster = Node(
        package='chair_detector',
        executable='cone_cluster_node',
        name='cone_cluster_node',
        output='screen',
    )

    # 【追加】凸包計算ノード
    cone_area = Node(
        package='chair_detector',
        executable='cone_area_node',
        name='cone_area_node',
        output='screen',
    )
    
    cone_color_detector = Node(
        package='chair_detector',
        executable='cone_color_detector_node',
        name='cone_color_detector_node',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(camera_node)
    ld.add_action(static_tf_lidar)
    ld.add_action(static_tf_camera)
    ld.add_action(scan_to_pointcloud)
    ld.add_action(cone_cluster) # 追加
    ld.add_action(cone_area)    # 追加
    ld.add_action(cone_color_detector)

    return ld
