cmake_minimum_required(VERSION 3.5)
project(cone_detector)

# ポリシーの設定
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

set(PCL_DIR "/usr/lib/cmake/pcl")  # 適切なPCLのCMakeディレクトリを指定

# 必要なROS2のパッケージを見つける
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
# 変更: PCLに 'surface' を追加 (凸包計算に必要)
find_package(PCL REQUIRED COMPONENTS common surface filters segmentation) 
find_package(rclcpp REQUIRED)
#find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)  # 追加
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)  # tf2を追加
find_package(tf2_ros REQUIRED)  # tf2_rosも追加
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # 追加 (PolygonStampedのため)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)


include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(leg_detector src/leg_detector_v2.cpp)
target_link_libraries(leg_detector 
  ${PCL_LIBRARIES}
)
ament_target_dependencies(leg_detector
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
)

add_executable(scan_to_pointcloud src/scan_to_pointcloud.cpp)
ament_target_dependencies(scan_to_pointcloud rclcpp sensor_msgs laser_geometry)

add_executable(chair_detector src/chair_detector_v3.cpp)
target_link_libraries(chair_detector 
  ${PCL_LIBRARIES}
)
ament_target_dependencies(chair_detector
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# --- ここから追加 ---

add_executable(cone_cluster_node src/cone_cluster_node.cpp)
target_link_libraries(cone_cluster_node 
  ${PCL_LIBRARIES}
)
ament_target_dependencies(cone_cluster_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

add_executable(cone_area_node src/cone_area_node.cpp)
target_link_libraries(cone_area_node 
  ${PCL_LIBRARIES}
)
ament_target_dependencies(cone_area_node
  rclcpp
  sensor_msgs
  pcl_conversions
  geometry_msgs
)

add_executable(cone_color_detector_node src/cone_color_detector_node.cpp)
ament_target_dependencies(cone_color_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  pcl_conversions
  image_transport
)
target_link_libraries(cone_color_detector_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${image_transport_LIBRARIES}
)

add_executable(cone_fusion_node src/cone_fusion_node.cpp)
ament_target_dependencies(cone_fusion_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_geometry_msgs
  image_geometry
  # message_filters (今回は使わなかった)
)
target_link_libraries(cone_fusion_node
  ${PCL_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

# (install(TARGETS ...) の中に追記)
install(TARGETS
  # ... (既存のノード) ...
  cone_color_detector_node
  cone_fusion_node  # これを追記
  DESTINATION lib/${PROJECT_NAME}
)

# --- ここまで追加 ---


# インストールの設定
install(TARGETS
  scan_to_pointcloud
  leg_detector
  chair_detector
  cone_cluster_node  # 追加
  cone_area_node     # 追加
  cone_color_detector_node  # これを追記
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
