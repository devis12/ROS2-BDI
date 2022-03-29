cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_on_webots)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ros2_bdi_core REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)
find_package(ros2_bdi_utils REQUIRED)
find_package(ros2_bdi_skills REQUIRED)
find_package(example_interfaces REQUIRED)

set(bdi_act_client_dep
  rclcpp
  plansys2_executor
  ros2_bdi_core
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
  example_interfaces
)

add_executable(move_gripper src/blocksworld/actions/move_gripper.cpp)
ament_target_dependencies(move_gripper ${bdi_act_client_dep})

add_executable(gripper_pickup src/blocksworld/actions/gripper_pickup.cpp)
ament_target_dependencies(gripper_pickup ${bdi_act_client_dep})

add_executable(gripper_putdown src/blocksworld/actions/gripper_putdown.cpp)
ament_target_dependencies(gripper_putdown ${bdi_act_client_dep})

install(TARGETS
  move_gripper
  gripper_pickup
  gripper_putdown
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
