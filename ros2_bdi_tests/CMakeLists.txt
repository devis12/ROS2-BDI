cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_tests)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ros2_bdi_core REQUIRED)
find_package(ros2_bdi_utils REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)

set(psys2_act_client_dep
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

set(sensors_dep
  rclcpp
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_core
)

add_executable(movetoward src/actions/cleaner_cli/movetoward.cpp)
ament_target_dependencies(movetoward ${psys2_act_client_dep})

add_executable(doclean src/actions/cleaner_cli/doclean.cpp)
ament_target_dependencies(doclean ${psys2_act_client_dep})

add_executable(recharge src/actions/cleaner_cli/recharge.cpp)
ament_target_dependencies(recharge ${psys2_act_client_dep})

add_executable(wp_sensor src/sensors/wp_sensor.cpp)
ament_target_dependencies(wp_sensor ${sensors_dep})


install(TARGETS
  movetoward
  recharge
  doclean

  wp_sensor

  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
