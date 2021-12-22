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
find_package(plansys2_problem_expert REQUIRED)

find_package(ros2_bdi_utils REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)
find_package(ros2_bdi_skills REQUIRED)
find_package(ros2_bdi_core REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


set(psys2_act_client_dep
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

set(bdi_act_client_dep
  rclcpp
  rclcpp_action
  plansys2_msgs
  plansys2_executor
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
  ros2_bdi_core
)

set(sensors_dep
  rclcpp
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
  ros2_bdi_core
)

add_executable(movetoward src/actions/generic_cli/generic_cli_raw/movetoward.cpp)
ament_target_dependencies(movetoward ${psys2_act_client_dep})

add_executable(recharge src/actions/generic_cli/generic_cli_raw/recharge.cpp)
ament_target_dependencies(recharge ${psys2_act_client_dep})

add_executable(doclean src/actions/cleaner_cli/cleaner_cli_raw/doclean.cpp)
ament_target_dependencies(doclean ${psys2_act_client_dep})

add_executable(dosweep src/actions/sweeper_cli/sweeper_cli_raw/dosweep.cpp)
ament_target_dependencies(dosweep ${psys2_act_client_dep})

add_executable(asksweeping src/actions/cleaner_cli/cleaner_cli_raw/asksweeping.cpp)
ament_target_dependencies(asksweeping ros2_bdi_interfaces ${psys2_act_client_dep})



add_executable(movetoward_bdi src/actions/generic_cli/generic_cli_bdi/movetoward.cpp)
ament_target_dependencies(movetoward_bdi ${bdi_act_client_dep})

add_executable(recharge_bdi src/actions/generic_cli/generic_cli_bdi/recharge.cpp)
ament_target_dependencies(recharge_bdi ${bdi_act_client_dep})

add_executable(doclean_bdi src/actions/cleaner_cli/cleaner_cli_bdi/doclean.cpp)
ament_target_dependencies(doclean_bdi ${bdi_act_client_dep})

add_executable(asksweeping_bdi src/actions/cleaner_cli/cleaner_cli_bdi/asksweeping.cpp)
ament_target_dependencies(asksweeping_bdi ${bdi_act_client_dep})

add_executable(dosweep_bdi src/actions/sweeper_cli/sweeper_cli_bdi/dosweep.cpp)
ament_target_dependencies(dosweep_bdi ${bdi_act_client_dep})

add_executable(wp_sensor src/sensors/wp_sensor.cpp)
ament_target_dependencies(wp_sensor ${sensors_dep})


install(TARGETS
  movetoward
  recharge
  doclean
  asksweeping
  dosweep

  movetoward_bdi
  recharge_bdi
  doclean_bdi
  asksweeping_bdi
  dosweep_bdi
  wp_sensor

  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
