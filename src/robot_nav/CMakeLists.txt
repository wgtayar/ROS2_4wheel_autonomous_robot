cmake_minimum_required(VERSION 3.8)
project(robot_nav)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1) Tell rosidl to generate your action interfaces:
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Navigate.action"
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
)

# 2) Export the runtime dependency so downstream packages pick up the include dirs
ament_export_dependencies(rosidl_default_runtime)

# 3) Make sure both your `include/` and the _binary_ directory (where those
#    generated headers live) are on your include path:
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

# 4) Declare your node & link against all of these:
add_executable(navigate_node src/navigate.cpp)
ament_target_dependencies(navigate_node
  rclcpp
  rclcpp_action
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  rosidl_default_runtime
)

install(TARGETS navigate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
