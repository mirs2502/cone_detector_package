import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # --- ★★★ use_sim_time をここで一括定義 ★★★ ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', # リアルロボットなので false
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    # --------------------------------------------------

    # --- mirs.launch.py の引数をここで定義 ---
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB1'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0'
    )

    # --- 1. mirs.launch.py をインクルード ---
    mirs_launch_file = os.path.join(
        get_package_share_directory('mirs'), 'launch', 'mirs.launch.py'
    )
    
    mirs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mirs_launch_file),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'use_sim_time': use_sim_time, # ★ use_sim_time を渡す
        }.items()
    )

    # --- 2. cone_detector 側のノードを起動 ---

    # (A) v4l2_camera ノード
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[
            {'video_device': '/dev/video2'},
            {'use_sim_time': use_sim_time} # ★ use_sim_time を渡す
        ], 
        remappings=[
            ('camera/image_raw', '/image_raw'),
            ('camera/camera_info', '/camera_info')
        ]
    )

    # (B) camera_link の TF
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments = ['0.0', '0.0', '0.5', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}] # ★ use_sim_time を渡す
    )

    # (C) scan_to_pointcloud ノード
    scan_to_pointcloud = Node(
        package='cone_detector',
        executable='scan_to_pointcloud',
        name='scan_to_pointcloud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # ★ use_sim_time を渡す
    )

    # (D) cone_cluster ノード
    cone_cluster = Node(
        package='cone_detector',
        executable='cone_cluster_node',
        name='cone_cluster_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # ★ use_sim_time を渡す
    )

    # (E) cone_color_detector ノード
    cone_color_detector = Node(
        package='cone_detector',
        executable='cone_color_detector_node',
        name='cone_color_detector_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # ★ use_sim_time を渡す
    )

    # (F) cone_fusion ノード
    cone_fusion = Node(
        package='cone_detector',
        executable='cone_fusion_node',
        name='cone_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # ★ use_sim_time を渡す
    )

    # (G) cone_area ノード
    cone_area = Node(
        package='cone_detector',
        executable='cone_area_node',
        name='cone_area_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # ★ use_sim_time を渡す
    )

    # --- 3. 起動するノードを登録 ---
    ld = LaunchDescription()
    
    # 引数の登録
    ld.add_action(use_sim_time_arg) # ★ 登録
    ld.add_action(serial_port_arg)
    ld.add_action(lidar_port_arg)
    
    # mirs.launch.py 本体
    ld.add_action(mirs_launch)
    
    # cone_detector のノード群
    ld.add_action(camera_node)
    ld.add_action(static_tf_camera)
    ld.add_action(scan_to_pointcloud)
    ld.add_action(cone_cluster)
    ld.add_action(cone_color_detector)
    ld.add_action(cone_fusion)
    ld.add_action(cone_area)

    return ld
