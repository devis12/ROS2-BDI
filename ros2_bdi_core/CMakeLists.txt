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

include_directories(include)

set(CORE-SUPPORT-SOURCES
  src/support/plansys2_monitor_client.cpp
  src/support/trigger_plan_client.cpp
)

add_library(${PROJECT_NAME} SHARED ${CORE-SUPPORT-SOURCES})
ament_target_dependencies(${PROJECT_NAME} rclcpp lifecycle_msgs ros2_bdi_interfaces)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

set(common_dependencies
  rclcpp
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
  plansys2_msgs
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
# link scheduler to ros2_bdi_core library since it makes usage of trigger_plan_client 
target_link_libraries(scheduler
    ${PROJECT_NAME}
)

add_executable(plan_director src/plan_director.cpp)
ament_target_dependencies(plan_director 
  ${common_dependencies} 
  plansys2_msgs
  ${pddl_experts}
  plansys2_executor
)

add_executable(plansys2_monitor src/plansys2_monitor.cpp)
ament_target_dependencies(plansys2_monitor 
  ${common_dependencies}
)
# link plansys2_monitor to ros2_bdi_core library since it makes usage of plansys2_monitor_client 
target_link_libraries(plansys2_monitor
    ${PROJECT_NAME}
)

add_executable(communications src/communications.cpp)
ament_target_dependencies(communications 
  ${common_dependencies}
)

add_executable(event_listener src/event_listener.cpp)
target_link_libraries(event_listener
  yaml-cpp  
)
ament_target_dependencies(event_listener 
  ${common_dependencies}
)


install(TARGETS
  belief_manager
  scheduler
  plan_director
  plansys2_monitor
  communications
  event_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
