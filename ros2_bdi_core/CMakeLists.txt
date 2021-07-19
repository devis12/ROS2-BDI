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

include_directories(include)

set(common_dependencies
  rclcpp
  plansys2_msgs
  ros2_bdi_utils ros2_bdi_interfaces
)

set(pddl_experts
  plansys2_domain_expert
  plansys2_problem_expert 
)

add_executable(belief_manager src/belief_manager.cpp)
ament_target_dependencies(belief_manager 
  ${common_dependencies}
  ${pddl_experts}
  std_msgs 
)


add_executable(scheduler src/scheduler.cpp)
ament_target_dependencies(scheduler 
  ${common_dependencies} 
  ${pddl_experts}
  plansys2_planner
)

add_executable(plan_director src/plan_director.cpp)
ament_target_dependencies(plan_director 
  ${common_dependencies} 
  plansys2_executor
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  belief_manager
  scheduler
  plan_director
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
