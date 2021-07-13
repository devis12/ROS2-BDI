#ifndef PDDLBDIConverter__UTILS_H_
#define PDDLBDIConverter__UTILS_H_

#include <string>
#include <vector>
#include <set>
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"

using std::string;
using std::vector;
using std::set;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::DesireSet;

namespace PDDLBDIConverter
{

  /*
    Convert PlanSys2 PDDL Instance to ROS2-BDI Belief
  */
  Belief convertPDDLInstance(const Instance instance);

  /*
    Convert PlanSys2 PDDL Instances to ROS2-BDI Beliefs
  */
  vector<Belief> convertPDDLInstances(const vector<Instance> instances);
  
  /*
    Convert PlanSys2 PDDL Predicate to ROS2-BDI Belief
  */
  Belief convertPDDLPredicate(const Predicate predicate);

  /*
    Convert PlanSys2 PDDL Predicates to ROS2-BDI Beliefs
  */
  vector<Belief> convertPDDLPredicates(const vector<Predicate> predicates);

  /*
    Convert PlanSys2 PDDL Function to ROS2-BDI Belief
  */
  Belief convertPDDLFunction(const Function function);

  /*
    Convert PlanSys2 PDDL Functions to ROS2-BDI Beliefs
  */
  vector<Belief> convertPDDLFunctions(const vector<Function> functions);

  /*
    Convert Desire into PDDL Goal
  */
  string desireToGoal(const Desire& desire);
  
}  // namespace PDDLBDIConverter

#endif  // PDDLBDIConverter__UTILS_H_