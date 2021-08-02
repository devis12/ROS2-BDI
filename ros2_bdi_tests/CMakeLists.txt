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
find_package(ros2_bdi_core REQUIRED)
find_package(ros2_bdi_utils REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(gazebo_msgs REQUIRED)

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

add_executable(movetoward src/actions/generic_cli/movetoward.cpp)
ament_target_dependencies(movetoward ${psys2_act_client_dep})

add_executable(recharge src/actions/generic_cli/recharge.cpp)
ament_target_dependencies(recharge ${psys2_act_client_dep})

add_executable(doclean src/actions/cleaner_cli/doclean.cpp)
ament_target_dependencies(doclean ${psys2_act_client_dep})

add_executable(asksweeping src/actions/cleaner_cli/asksweeping.cpp)
ament_target_dependencies(asksweeping ros2_bdi_interfaces ${psys2_act_client_dep})

add_executable(waitsweeping src/actions/cleaner_cli/waitsweeping.cpp)
ament_target_dependencies(waitsweeping ros2_bdi_interfaces ${psys2_act_client_dep})

add_executable(dosweep src/actions/sweeper_cli/dosweep.cpp)
ament_target_dependencies(dosweep ${psys2_act_client_dep})

#add_executable(doclean_gazebo src/actions/cleaner_gazebo/doclean_gazebo.cpp)
#ament_target_dependencies(doclean_gazebo geometry_msgs ${psys2_act_client_dep})

#add_executable(movetoward_gazebo src/actions/cleaner_gazebo/movetoward_gazebo.cpp)
#ament_target_dependencies(movetoward_gazebo geometry_msgs sensor_msgs gazebo_msgs plansys2_problem_expert ${psys2_act_client_dep})

add_executable(wp_sensor src/sensors/wp_sensor.cpp)
ament_target_dependencies(wp_sensor ${sensors_dep})


install(TARGETS
  movetoward
  recharge
  doclean
  asksweeping
  waitsweeping
  dosweep
  #doclean_gazebo
  #movetoward_gazebo

  wp_sensor

  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
