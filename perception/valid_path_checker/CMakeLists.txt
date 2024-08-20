cmake_minimum_required(VERSION 3.8)
project(valid_path_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(
  valid_path_checker_node SHARED
  src/valid_path_checker_node.cpp
)

ament_target_dependencies(
  valid_path_checker_node
  rclcpp
  rclcpp_components
  autoware_auto_planning_msgs
  autoware_auto_perception_msgs
  perception_msgs
  geometry_msgs
)

rclcpp_components_register_nodes(
  valid_path_checker_node
  "perception_component::ValidPathChecker"
)

install(TARGETS 
valid_path_checker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
