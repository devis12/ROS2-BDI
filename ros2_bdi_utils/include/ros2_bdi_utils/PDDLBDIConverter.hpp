#ifndef PDDLBDIConverter__UTILS_H_
#define PDDLBDIConverter__UTILS_H_

#include <string>
#include <vector>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"


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
    Convert Desire into PDDL Goal
  */
  std::string desireToGoal(const ros2_bdi_interfaces::msg::Desire& desire);
  
}  // namespace PDDLBDIConverter

#endif  // PDDLBDIConverter__UTILS_H_