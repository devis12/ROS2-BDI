cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_on_webots)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)


install(DIRECTORY
  launch
  worlds
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
