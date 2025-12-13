import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # --- 引数の定義 ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')

    # --- 1. mirs.launch.py (ロボット本体 + LiDARドライバ) ---
    mirs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mirs'), 'launch', 'mirs.launch.py')
        ),
        launch_arguments={
            'serial_port': serial_port,
            'lidar_port': lidar_port,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # --- 2. 点群変換 (scan_to_pointcloud) ---
    scan_to_pointcloud = Node(
        package='cone_detector',
        executable='scan_to_pointcloud',
        name='scan_to_pointcloud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 3. クラスタリング (cone_cluster_node) ---
    # 円弧形状の抽出を行うノード -> /cone_centers
    cone_cluster = Node(
        package='cone_detector',
        executable='cone_cluster_node',
        name='cone_cluster_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 4. コーン記憶 (cone_accumulator) ---
    # /cone_centers を受け取り、地図上に蓄積して /accumulated_cones を出す
    cone_accumulator = Node(
        package='coverage_planner',
        executable='cone_accumulator',
        name='cone_accumulator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 5. 凸包作成 (cone_area_node) ---
    # 入力を「記憶したコーン(/accumulated_cones)」に変更
    cone_area = Node(
        package='cone_detector',
        executable='cone_area_node',
        name='cone_area_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/confirmed_cones', '/accumulated_cones')
        ]
    )

    # --- 6. 経路生成 (zigzag_generator) ---
    # 凸包を受け取り、ジグザグ経路を生成
    zigzag_generator = Node(
        package='coverage_planner',
        executable='zigzag_generator',
        name='zigzag_generator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # --- 7. 【新規】ランドマーク自己位置推定 (landmark_localizer) ---
    landmark_localizer = Node(
        package='cone_detector', # setup.pyがあるパッケージ名
        executable='landmark_localizer.py',
        name='landmark_localizer',
        output='screen',
        parameters=[{'min_shared_landmarks': 1}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        mirs_launch,
        scan_to_pointcloud,
        cone_cluster,
        cone_accumulator,
        cone_area,
        zigzag_generator,
        landmark_localizer
    ])
