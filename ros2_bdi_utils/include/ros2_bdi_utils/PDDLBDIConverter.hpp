#ifndef PDDLBDIConverter__UTILS_H_
#define PDDLBDIConverter__UTILS_H_

#include <vector>
#include <set>
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"

using std::vector;
using std::set;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;

namespace PDDLBDIConverter
{

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
    Extract from passed set of ManagedBelief objects a BeliefSet msg
  */
  BeliefSet extractBeliefSetMsg(const set<ManagedBelief> managed_beliefs);

    /*
    Extract from passed vector just beliefs of type predicate
  */
  vector<Belief> extractPredicates(const vector<Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type fluent
  */
  vector<Belief> extractFluents(const vector<Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type predicate and put it into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGPredicates(const vector<Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type fluent and put it into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGFluents(const vector<Belief> beliefs);

  /*
    Extract from passed set just beliefs of type predicate and put it into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGPredicates(const set<ManagedBelief> managed_beliefs);

  /*
    Extract from passed set just beliefs of type fluent and put it into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGFluents(const set<ManagedBelief> managed_beliefs);
  
  
}  // namespace PDDLBDIConverter

#endif  // PDDLBDIConverter__UTILS_H_