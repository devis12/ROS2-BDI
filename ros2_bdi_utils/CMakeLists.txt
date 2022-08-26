cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_utils)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)

include_directories(include)

set(UTILS-SOURCES
  src/PDDLBDIConverter.cpp
  src/BDIPDDLConverter.cpp
  src/BDIFilter.cpp
  src/PDDLUtils.cpp

  src/ManagedBelief.cpp
  src/ManagedDesire.cpp
  src/ManagedCondition.cpp
  src/ManagedConditionsConjunction.cpp
  src/ManagedConditionsDNF.cpp
  src/ManagedPlan.cpp
  src/ManagedReactiveRule.cpp

  src/BDIYAMLParser.cpp
)

add_library(${PROJECT_NAME} SHARED ${UTILS-SOURCES})
ament_target_dependencies(${PROJECT_NAME} plansys2_msgs plansys2_domain_expert plansys2_problem_expert plansys2_planner ros2_bdi_interfaces)

install(DIRECTORY include/
  DESTINATION include/
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp  
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

ament_package()
