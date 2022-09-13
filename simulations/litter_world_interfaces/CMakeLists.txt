cmake_minimum_required(VERSION 3.5)
project(litter_world_interfaces)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/Pose.msg"
  "msg/GridRowStatus.msg"
  "msg/GridStatus.msg"

  "action/CmdPose.action"
  "action/CmdLoad.action"
)


ament_package()
