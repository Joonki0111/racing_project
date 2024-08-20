cmake_minimum_required(VERSION 3.8)
project(lanechange_trigger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tier4_rtc_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(perception_msgs REQUIRED)

include_directories(include)

add_library(
  lanechange_trigger_node SHARED
  src/lanechange_trigger_node.cpp
)

ament_target_dependencies(
  lanechange_trigger_node
  rclcpp
  rclcpp_components
  std_msgs
  tier4_rtc_msgs
  example_interfaces
  perception_msgs
)

rclcpp_components_register_nodes(
  lanechange_trigger_node
  "planning_component::LanechangeTrigger"
)

install(TARGETS 
  lanechange_trigger_node
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
