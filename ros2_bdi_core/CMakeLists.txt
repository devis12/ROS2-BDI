cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_core)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ros2_bdi_utils REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)

find_library(yaml-cpp REQUIRED)

set(common_dependencies
  rclcpp
  plansys2_msgs
  lifecycle_msgs
  ros2_bdi_utils ros2_bdi_interfaces
)

set(pddl_experts
  plansys2_domain_expert
  plansys2_problem_expert 
)

add_executable(belief_manager src/belief_manager.cpp)
target_link_libraries(belief_manager
  yaml-cpp  
)
ament_target_dependencies(belief_manager 
  ${common_dependencies}
  ${pddl_experts}
  std_msgs 
)


add_executable(scheduler src/scheduler.cpp)
target_link_libraries(scheduler
  yaml-cpp  
)
ament_target_dependencies(scheduler 
  ${common_dependencies} 
  ${pddl_experts}
  plansys2_planner
)

add_executable(plan_director src/plan_director.cpp)
ament_target_dependencies(plan_director 
  ${common_dependencies} 
  ${pddl_experts}
  plansys2_executor
)

add_executable(plansys2_monitor src/plansys2_monitor.cpp)
ament_target_dependencies(plansys2_monitor 
  rclcpp
  ros2_bdi_interfaces
  lifecycle_msgs
)

add_executable(communications src/communications.cpp)
ament_target_dependencies(communications 
  rclcpp
  ros2_bdi_interfaces
  ros2_bdi_utils
)

# install(DIRECTORY include/
#   DESTINATION include/
# )

install(TARGETS
  belief_manager
  scheduler
  plan_director
  plansys2_monitor
  communications
  DESTINATION lib/${PROJECT_NAME}
)

# ament_export_include_directories(include)

ament_package()
