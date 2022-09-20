cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_on_webots)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ros2_bdi_core REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)
find_package(ros2_bdi_utils REQUIRED)
find_package(ros2_bdi_skills REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(javaff_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(webots_ros2_simulations_interfaces REQUIRED)

set(bdi_act_client_dep
  rclcpp
  plansys2_executor
  ros2_bdi_core
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
  javaff_interfaces
)

set(bdi_sensor_dep
  rclcpp
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
)


add_executable(carrier_move_sensor src/blocksworld/carrier/sensors/carrier_move_sensor.cpp)
ament_target_dependencies(carrier_move_sensor ${bdi_sensor_dep} webots_ros2_simulations_interfaces)

add_executable(carrier_moving_boxes_sensor src/blocksworld/carrier/sensors/carrier_moving_boxes_sensor.cpp)
ament_target_dependencies(carrier_moving_boxes_sensor ${bdi_sensor_dep} geometry_msgs)

add_executable(carrier_move src/blocksworld/carrier/actions/carrier_move.cpp)
ament_target_dependencies(carrier_move ${bdi_act_client_dep} example_interfaces geometry_msgs webots_ros2_simulations_interfaces)

add_executable(carrier_unload src/blocksworld/carrier/actions/carrier_unload.cpp)
ament_target_dependencies(carrier_unload ${bdi_act_client_dep})

add_executable(gripper_move src/blocksworld/gripper/actions/gripper_move.cpp)
ament_target_dependencies(gripper_move ${bdi_act_client_dep} example_interfaces geometry_msgs webots_ros2_simulations_interfaces)

add_executable(gripper_pickup src/blocksworld/gripper/actions/gripper_pickup.cpp)
ament_target_dependencies(gripper_pickup ${bdi_act_client_dep} example_interfaces webots_ros2_simulations_interfaces)

add_executable(gripper_putdown src/blocksworld/gripper/actions/gripper_putdown.cpp)
ament_target_dependencies(gripper_putdown ${bdi_act_client_dep} example_interfaces webots_ros2_simulations_interfaces)

add_executable(gripper_put_on_carrier src/blocksworld/gripper/actions/gripper_put_on_carrier.cpp)
ament_target_dependencies(gripper_put_on_carrier ${bdi_act_client_dep} example_interfaces webots_ros2_simulations_interfaces)

add_executable(req_carrier_to_come src/blocksworld/gripper/actions/req_carrier_to_come.cpp)
ament_target_dependencies(req_carrier_to_come ${bdi_act_client_dep})

add_executable(gripper_get_carriers_status src/blocksworld/gripper/sensors/gripper_get_carriers_status.cpp)
ament_target_dependencies(gripper_get_carriers_status ${bdi_act_client_dep} webots_ros2_simulations_interfaces)

install(TARGETS
  #carrier sensors
  carrier_move_sensor
  carrier_moving_boxes_sensor
  #carrier actions
  carrier_move
  carrier_unload
  
  #gripper sensors
  gripper_get_carriers_status
  
  #gripper actions
  gripper_move
  gripper_pickup
  gripper_putdown
  gripper_put_on_carrier
  req_carrier_to_come
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
