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
find_package(javaff_interfaces REQUIRED)

find_library(yaml-cpp REQUIRED)

include_directories(include)

set(CORE-LIB-SOURCES
  src/support/plansys_monitor_client.cpp
  src/support/trigger_plan_client.cpp

  src/scheduler.cpp
  
  src/support/javaff_client.cpp
)

set(common_dependencies
  rclcpp
  ros2_bdi_utils ros2_bdi_interfaces
)

set(pddl_experts
  plansys2_domain_expert
  plansys2_problem_expert 
)


add_library(${PROJECT_NAME} SHARED ${CORE-LIB-SOURCES})
ament_target_dependencies(${PROJECT_NAME} 
  lifecycle_msgs   
  ${common_dependencies} 
  ${pddl_experts}
  plansys2_planner
  javaff_interfaces
)
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

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

add_executable(plansys_monitor src/plansys_monitor.cpp)
ament_target_dependencies(plansys_monitor 
  ${common_dependencies}
)
#link plansys_monitor to ros2_bdi_core library since it makes usage of plansys_monitor_client 
target_link_libraries(plansys_monitor
    ${PROJECT_NAME}
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
# link plansys_monitor to ros2_bdi_core library since it makes usage of plansys_monitor_client 
target_link_libraries(belief_manager
  ${PROJECT_NAME}
)

add_executable(scheduler_offline src/scheduler_offline.cpp)
ament_target_dependencies(scheduler_offline 
  ${common_dependencies} 
  ${pddl_experts}
  plansys2_planner
)
# link scheduler_offline to ros2_bdi_core library since it makes usage of trigger_plan_client 
target_link_libraries(scheduler_offline
    ${PROJECT_NAME}
)

add_executable(scheduler_online src/scheduler_online.cpp)
ament_target_dependencies(scheduler_online 
  ${common_dependencies} 
  ${pddl_experts}
  plansys2_planner
  plansys2_executor
  javaff_interfaces
)
# link scheduler_offline to ros2_bdi_core library since it makes usage of trigger_plan_client 
target_link_libraries(scheduler_online
    ${PROJECT_NAME}
)

add_executable(plan_director src/plan_director.cpp)
ament_target_dependencies(plan_director 
  ${common_dependencies} 
  plansys2_msgs
  ${pddl_experts}
  plansys2_executor
)
# link plansys_monitor to ros2_bdi_core library since it makes usage of plansys_monitor_client 
target_link_libraries(plan_director
    ${PROJECT_NAME}
)

add_executable(ma_request_handler src/ma_request_handler.cpp)
ament_target_dependencies(ma_request_handler 
  ${common_dependencies}
)
# link plansys_monitor to ros2_bdi_core library since it makes usage of plansys_monitor_client 
target_link_libraries(ma_request_handler
    ${PROJECT_NAME}
)

add_executable(event_listener src/event_listener.cpp)
target_link_libraries(event_listener
  yaml-cpp  
)
ament_target_dependencies(event_listener 
  ${common_dependencies}
)
# link plansys_monitor to ros2_bdi_core library since it makes usage of plansys_monitor_client 
target_link_libraries(event_listener
    ${PROJECT_NAME}
)


install(TARGETS
  plansys_monitor
  belief_manager
  #scheduler
  scheduler_offline
  scheduler_online
  plan_director
  ma_request_handler
  event_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
