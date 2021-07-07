cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_interfaces)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/Belief.msg"
  "msg/Desire.msg"
  "msg/Condition.msg"
  "msg/Plan.msg"
  DEPENDENCIES plansys2_msgs
)


ament_package()
