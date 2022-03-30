cmake_minimum_required(VERSION 3.5)
project(webots_ros2_simulations_interfaces)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/MoveStatus.msg"

  DEPENDENCIES geometry_msgs
)


ament_package()
