cmake_minimum_required(VERSION 3.8)
project(robot_nav)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoalInput.srv"
  DEPENDENCIES std_msgs
)

# Add include directories
include_directories(include)

# For the node library
add_library(nav2goal_lib
  src/nav2goal.cpp
)

ament_target_dependencies(nav2goal_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

rosidl_target_interfaces(nav2goal_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# For the wandering node library
add_library(nav_lib
  src/nav.cpp
)

ament_target_dependencies(nav_lib
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

rosidl_target_interfaces(nav_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# For the executable nav2goal node
add_executable(nav2goal_node src/nav2goal_node.cpp)
target_link_libraries(nav2goal_node nav2goal_lib)

ament_target_dependencies(nav2goal_node
  rclcpp
)

# For the executable wandering node
add_executable(nav_node src/nav_node.cpp)
target_link_libraries(nav_node nav_lib)

ament_target_dependencies(nav_node
  rclcpp
)

# Install
install(TARGETS
  nav2goal_lib
  nav2goal_node
  nav_lib
  nav_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

ament_package()