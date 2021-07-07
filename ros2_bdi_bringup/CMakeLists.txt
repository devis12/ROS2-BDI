cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_bringup)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
