cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_core)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ros2_bdi_utils REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)

add_executable(belief_manager src/belief_manager.cpp)
ament_target_dependencies(belief_manager 
  rclcpp 
  plansys2_msgs plansys2_problem_expert plansys2_domain_expert
  std_msgs 
  ros2_bdi_utils ros2_bdi_interfaces
)

install(TARGETS
  belief_manager
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
