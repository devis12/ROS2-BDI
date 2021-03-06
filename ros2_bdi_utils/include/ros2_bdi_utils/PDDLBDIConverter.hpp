#ifndef PDDLBDIConverter__UTILS_H_
#define PDDLBDIConverter__UTILS_H_

#include <string>
#include <vector>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_msgs/msg/plan_item.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/bdi_action_execution_info.hpp"


namespace PDDLBDIConverter
{

  /*
    Convert PlanSys2 PDDL Instance to ROS2-BDI Belief
  */
  ros2_bdi_interfaces::msg::Belief convertPDDLInstance(const plansys2::Instance instance);

  /*
    Convert PlanSys2 PDDL Instances to ROS2-BDI Beliefs
  */
  std::vector<ros2_bdi_interfaces::msg::Belief> convertPDDLInstances(const std::vector<plansys2::Instance> instances);
  
  /*
    Convert PlanSys2 PDDL Predicate to ROS2-BDI Belief
  */
  ros2_bdi_interfaces::msg::Belief convertPDDLPredicate(const plansys2::Predicate predicate);

  /*
    Convert PlanSys2 PDDL Predicates to ROS2-BDI Beliefs
  */
  std::vector<ros2_bdi_interfaces::msg::Belief> convertPDDLPredicates(const std::vector<plansys2::Predicate> predicates);

  /*
    Convert PlanSys2 PDDL Function to ROS2-BDI Belief
  */
  ros2_bdi_interfaces::msg::Belief convertPDDLFunction(const plansys2::Function function);

  /*
    Convert PlanSys2 PDDL Functions to ROS2-BDI Beliefs
  */
  std::vector<ros2_bdi_interfaces::msg::Belief> convertPDDLFunctions(const std::vector<plansys2::Function> functions);

  /*
    get index of action with given action_name & args in the vector<PlanItem> in current_plan_.body, -1 if not present
    action_full_name is in the form "(a1 p1 p2 p3):timex1000"
  */
  int getActionIndex(const std::vector<plansys2_msgs::msg::PlanItem>& current_plan_body, const std::string& action_full_name);


  /*
    get index of action with given action_name & args in the vector<ActionExecutionInfo> in psys2_plan_exec_info, -1 if not present
    action_full_name is in the form "(a1 p1 p2 p3):timex1000"
  */
  int getActionIndex(const std::vector<plansys2_msgs::msg::ActionExecutionInfo>& psys2_plan_exec_info, const std::string& action_full_name);
  
  /*
    Build a BDIActionExecutionInfo from the corresponding PlanSys2 ActionExecutionInfo
    PlanSys2 plan body is needed too (PlanItem array) to get the index of the executing action within it

    Timestamps of corresponding plan start are passed too 
  */
  ros2_bdi_interfaces::msg::BDIActionExecutionInfo buildBDIActionExecutionInfo(
    const std::optional<plansys2_msgs::msg::ActionExecutionInfo>& psys2_action_feed, 
    const std::vector<plansys2_msgs::msg::PlanItem>& current_plan_body,
    const int& action_index, 
    const int& first_ts_plan_sec, const unsigned int& first_ts_plan_nanosec);

  
}  // namespace PDDLBDIConverter

#endif  // PDDLBDIConverter__UTILS_H_