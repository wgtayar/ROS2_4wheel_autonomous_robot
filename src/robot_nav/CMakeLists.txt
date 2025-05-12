cmake_minimum_required(VERSION 3.8)
project(robot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate both service and action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Navigate.srv"
  "action/Navigate.action"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include
  # ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_executable(navigate_node
  src/navigate_node.cpp
  src/navigate.cpp
)
ament_target_dependencies(navigate_node
  rclcpp
  rclcpp_action
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp
)

target_link_libraries(navigate_node
  robot_nav__rosidl_typesupport_cpp
)

install(TARGETS navigate_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
