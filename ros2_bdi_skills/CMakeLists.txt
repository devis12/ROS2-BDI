cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_skills)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)
find_package(ros2_bdi_utils REQUIRED)

include_directories(include)

set(SKILLS-SOURCES
  src/sensor.cpp
  src/bdi_action_executor.cpp
)

add_library(${PROJECT_NAME} SHARED ${SKILLS-SOURCES})
ament_target_dependencies(${PROJECT_NAME} rclcpp ros2_bdi_interfaces ros2_bdi_utils plansys2_executor)

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
