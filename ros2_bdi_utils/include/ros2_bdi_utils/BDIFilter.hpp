#ifndef BDIFILTER__UTILS_H_
#define BDIFILTER__UTILS_H_

#include <string>
#include <vector>
#include <set>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedConditionsConjunction.hpp"
#include "ros2_bdi_utils/ManagedConditionsDNF.hpp"

namespace BDIFilter
{

    /*
    Extract from passed set of ManagedBelief objects a BeliefSet msg
  */
  ros2_bdi_interfaces::msg::BeliefSet extractBeliefSetMsg(const std::set<BDIManaged::ManagedBelief> managed_beliefs);

  /*
    Extract from passed set of ManagedDesire objects a DesireSet msg
  */
  ros2_bdi_interfaces::msg::DesireSet extractDesireSetMsg(const std::set<BDIManaged::ManagedDesire> managed_desires);

  /*
    Extract from passed vector just beliefs of type instance
  */
  std::vector<ros2_bdi_interfaces::msg::Belief> extractInstances(const std::vector<ros2_bdi_interfaces::msg::Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type predicate
  */
  std::vector<ros2_bdi_interfaces::msg::Belief> extractPredicates(const std::vector<ros2_bdi_interfaces::msg::Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type function
  */
  std::vector<ros2_bdi_interfaces::msg::Belief> extractFunctions(const std::vector<ros2_bdi_interfaces::msg::Belief> beliefs);

  /*
    Extract from passed vector beliefs and put them into a set of ManagedBelief objects
  */
  std::set<BDIManaged::ManagedBelief> extractMGBeliefs(const std::vector<ros2_bdi_interfaces::msg::Belief> beliefs);

  /*
    Extract from passed vector desires and put them into a set of ManagedDesire objects
  */
  std::set<BDIManaged::ManagedDesire> extractMGDesires(const std::vector<ros2_bdi_interfaces::msg::Desire> desires);

  /*
    Extract from passed vector beliefs and put them into a set of ManagedBelief objects
  */
  std::set<BDIManaged::ManagedBelief> extractMGInstances(const std::vector<ros2_bdi_interfaces::msg::Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type predicate and put them into a set of ManagedBelief objects
  */
  std::set<BDIManaged::ManagedBelief> extractMGPredicates(const std::vector<ros2_bdi_interfaces::msg::Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type function and put them into a set of ManagedBelief objects
  */
  std::set<BDIManaged::ManagedBelief> extractMGFunctions(const std::vector<ros2_bdi_interfaces::msg::Belief> beliefs);

  /*
    Extract from passed set just beliefs of type predicate and put them into a set of ManagedBelief objects
  */
  std::set<BDIManaged::ManagedBelief> extractMGInstances(const std::set<BDIManaged::ManagedBelief> managed_beliefs);
  
  /*
    Extract from passed set just beliefs of type predicate and put them into a set of ManagedBelief objects
  */
  std::set<BDIManaged::ManagedBelief> extractMGPredicates(const std::set<BDIManaged::ManagedBelief> managed_beliefs);

  /*
    Extract from passed set just beliefs of type function and put them into a set of ManagedBelief objects
  */
  std::set<BDIManaged::ManagedBelief> extractMGFunctions(const std::set<BDIManaged::ManagedBelief> managed_beliefs);

  /*
    Given array of ManagedCondition, desire base name (added a counter as suffix to distinguish them among each other),
    desire priority, desire deadline use it to build a ManagedDesire putting as value the conditions
    NOTE: for now just work around with condition(s) containing PREDICATE type as values,
    if @conditions do not contain any Belief with PREDICATE type, returns empty array
  */      
  std::vector<BDIManaged::ManagedDesire> conditionsToMGDesire(const BDIManaged::ManagedConditionsDNF& conditionsDNF, 
                  const std::string& desireBaseName, const float& desirePriority, const float& desireDeadline);

  /*
    Extract managed belief instances, filtering by type if provided
  */
  std::set<BDIManaged::ManagedBelief> filterMGBeliefInstances(const std::set<BDIManaged::ManagedBelief>& belief_set, 
    const BDIManaged::ManagedType& type = BDIManaged::ManagedType{"", std::nullopt});
  
}  // namespace BDIFilter

#endif  // BDIFILTER__UTILS_H_