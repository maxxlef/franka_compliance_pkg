cmake_minimum_required(VERSION 3.10)
project(arm_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Node
# interactive marker node
add_executable(interactive_marker_node src/interactive_marker_node.cpp)
ament_target_dependencies(interactive_marker_node
  rclcpp
  interactive_markers
  visualization_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  )
install(TARGETS interactive_marker_node RUNTIME DESTINATION lib/${PROJECT_NAME})

#force bridge node
add_executable(force_bridge_node src/force_bridge_node.cpp)
ament_target_dependencies(force_bridge_node 
  rclcpp 
  franka_msgs 
  geometry_msgs   
  tf2_ros
  tf2
  tf2_geometry_msgs
  )
install(TARGETS force_bridge_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# Wrench service node
add_executable(wrench_service_node src/wrench_service_node.cpp)
ament_target_dependencies(wrench_service_node
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  geometry_msgs
  )
install(TARGETS wrench_service_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# Motion smoother node
add_executable(motion_smoother_node src/motion_smoother_node.cpp)
ament_target_dependencies(motion_smoother_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  )
install(TARGETS motion_smoother_node RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
