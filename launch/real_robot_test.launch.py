import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    mirs_pkg = get_package_share_directory('mirs')
    cone_detector_pkg = get_package_share_directory('cone_detector')

    # 1. mirs.launch.py (Hardware, EKF, URDF)
    mirs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mirs_pkg, 'launch', 'mirs.launch.py')
        )
    )

    # 2. cone_detection.launch.py (Cone Detection, No Static TF)
    cone_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cone_detector_pkg, 'launch', 'cone_detection.launch.py')
        )
    )

    # 3. landmark_localizer (Note: renamed in CMakeLists.txt)
    landmark_localizer_node = Node(
        package='cone_detector',
        executable='landmark_localizer',
        name='landmark_localizer',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(mirs_launch)
    ld.add_action(cone_detection_launch)
    ld.add_action(landmark_localizer_node)

    return ld
