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
find_package(geometry_msgs REQUIRED)
find_package(webots_ros2_simulations_interfaces REQUIRED)

set(bdi_act_client_dep
  rclcpp
  plansys2_executor
  ros2_bdi_core
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
)

set(bdi_sensor_dep
  rclcpp
  ros2_bdi_interfaces
  ros2_bdi_utils
  ros2_bdi_skills
)


add_executable(carrier_move_sensor src/blocksworld/carrier/carrier_move_sensor.cpp)
ament_target_dependencies(carrier_move_sensor ${bdi_sensor_dep} webots_ros2_simulations_interfaces)

add_executable(carrier_move src/blocksworld/carrier/carrier_move.cpp)
ament_target_dependencies(carrier_move ${bdi_act_client_dep} example_interfaces geometry_msgs webots_ros2_simulations_interfaces)

add_executable(gripper_move src/blocksworld/gantry/gripper_move.cpp)
ament_target_dependencies(gripper_move ${bdi_act_client_dep} example_interfaces geometry_msgs webots_ros2_simulations_interfaces)

add_executable(gripper_pickup src/blocksworld/gantry/gripper_pickup.cpp)
ament_target_dependencies(gripper_pickup ${bdi_act_client_dep} example_interfaces)

add_executable(gripper_putdown src/blocksworld/gantry/gripper_putdown.cpp)
ament_target_dependencies(gripper_putdown ${bdi_act_client_dep} example_interfaces)

add_executable(gripper_put_on_carrier src/blocksworld/gantry/gripper_put_on_carrier.cpp)
ament_target_dependencies(gripper_put_on_carrier ${bdi_act_client_dep} example_interfaces)

add_executable(req_carrier_to_come src/blocksworld/gantry/req_carrier_to_come.cpp)
ament_target_dependencies(req_carrier_to_come ${bdi_act_client_dep})

add_executable(req_carrier_to_go src/blocksworld/gantry/req_carrier_to_go.cpp)
ament_target_dependencies(req_carrier_to_go ${bdi_act_client_dep})

install(TARGETS
  carrier_move_sensor
  carrier_move
  
  gripper_move
  gripper_pickup
  gripper_putdown
  gripper_put_on_carrier
  req_carrier_to_go
  req_carrier_to_come
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
