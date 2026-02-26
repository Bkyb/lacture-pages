cmake_minimum_required(VERSION 3.8)
project(manipulator_practice_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dsr_msgs2 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# Find Octomap explicitly
find_package(octomap REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
# TF
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  DEPENDENCIES geometry_msgs dsr_msgs2
)

add_executable(static_map_scanner src/static_map_scanner.cpp)
target_include_directories(static_map_scanner PUBLIC ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(static_map_scanner ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

ament_target_dependencies(static_map_scanner
  rclcpp
  std_srvs
  sensor_msgs
  dsr_msgs2
  pcl_conversions
  pcl_ros
  octomap_ros
  octomap_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_eigen
)

add_executable(robot_action_server src/robot_action_server.cpp)
target_include_directories(robot_action_server PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(robot_action_server
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(robot_action_server "${cpp_typesupport_target}")

install(TARGETS static_map_scanner robot_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
