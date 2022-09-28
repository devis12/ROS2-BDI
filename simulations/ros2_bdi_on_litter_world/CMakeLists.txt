cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_on_litter_world)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(litter_world_interfaces REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ros2_bdi_core REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)
find_package(ros2_bdi_utils REQUIRED)
find_package(ros2_bdi_skills REQUIRED)
# find_package(javaff_interfaces REQUIRED)

include_directories(include)

set(CORE-LIB-SOURCES
  src/litter_world/recycling_agent/actions/recycle.cpp
)

set(bdi_act_client_dep
  rclcpp
  plansys2_executor
  ros2_bdi_core
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
  # javaff_interfaces
)

set(bdi_sensor_dep
  rclcpp
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
)

add_library(${PROJECT_NAME} SHARED ${CORE-LIB-SOURCES})
ament_target_dependencies(${PROJECT_NAME} 
  ${bdi_act_client_dep}
  litter_world_interfaces
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

add_executable(move src/litter_world/recycling_agent/actions/move.cpp)
ament_target_dependencies(move ${bdi_act_client_dep} litter_world_interfaces)

add_executable(litter_pickup src/litter_world/recycling_agent/actions/litter_pickup.cpp)
ament_target_dependencies(litter_pickup ${bdi_act_client_dep} litter_world_interfaces)

add_executable(recycle_paper src/litter_world/recycling_agent/actions/recycle_paper.cpp)
ament_target_dependencies(recycle_paper ${bdi_act_client_dep} litter_world_interfaces)
target_link_libraries(recycle_paper
    ${PROJECT_NAME}
)

add_executable(recycle_plastic src/litter_world/recycling_agent/actions/recycle_plastic.cpp)
ament_target_dependencies(recycle_plastic ${bdi_act_client_dep} litter_world_interfaces)
target_link_libraries(recycle_plastic
    ${PROJECT_NAME}
)

add_executable(load_map_sensor src/litter_world/recycling_agent/sensors/load_map_sensor.cpp)
ament_target_dependencies(load_map_sensor ${bdi_sensor_dep} litter_world_interfaces)

add_executable(agent_area_sensor src/litter_world/recycling_agent/sensors/agent_area_sensor.cpp)
ament_target_dependencies(agent_area_sensor ${bdi_sensor_dep} litter_world_interfaces)

install(TARGETS
  move
  litter_pickup
  recycle_paper
  recycle_plastic

  load_map_sensor
  agent_area_sensor
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
