cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_interfaces)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/Belief.msg"
  "msg/BeliefSet.msg"
  "msg/Desire.msg"
  "msg/DesireBoost.msg"
  "msg/DesireSet.msg"
  "msg/Condition.msg"
  "msg/ConditionsConjunction.msg"
  "msg/ConditionsDNF.msg"
  "msg/BDIPlan.msg"
  "msg/BDIActionExecutionInfo.msg"
  "msg/BDIPlanExecutionInfo.msg"
  "msg/PlanningSystemState.msg"
  "msg/LifecycleStatus.msg"
  
  "srv/IsAcceptedOperation.srv"
  "srv/CheckBelief.srv"
  "srv/UpdBeliefSet.srv"
  "srv/CheckDesire.srv"
  "srv/UpdDesireSet.srv"
  "srv/BDIPlanExecution.srv"

  DEPENDENCIES plansys2_msgs
)


ament_package()
