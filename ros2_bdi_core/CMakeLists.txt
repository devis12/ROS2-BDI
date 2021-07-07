cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_core)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

#[[
add_executable(define_problem src/problem_experts/define_problem.cpp)
ament_target_dependencies(define_problem ${psys2_problem_exp_dep} plansys2_pddl_parser plansys2_msgs)
]]

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
