cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_utils)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)

include_directories(include)

set(UTILS-SOURCES
  src/PDDLBDIConverter.cpp
  src/BDIComparisons.cpp
  src/ManagedBelief.cpp
)

add_library(${PROJECT_NAME} SHARED ${UTILS-SOURCES})
ament_target_dependencies(${PROJECT_NAME} plansys2_msgs plansys2_problem_expert ros2_bdi_interfaces)

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

ament_package()
