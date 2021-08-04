#ifndef BDIFILTER__UTILS_H_
#define BDIFILTER__UTILS_H_

#include <vector>
#include <set>

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"

using std::vector;
using std::set;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::DesireSet;

namespace BDIFilter
{

    /*
    Extract from passed set of ManagedBelief objects a BeliefSet msg
  */
  BeliefSet extractBeliefSetMsg(const set<ManagedBelief> managed_beliefs);

  /*
    Extract from passed set of ManagedDesire objects a DesireSet msg
  */
  DesireSet extractDesireSetMsg(const set<ManagedDesire> managed_desires);

  /*
    Extract from passed vector just beliefs of type instance
  */
  vector<Belief> extractInstances(const vector<Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type predicate
  */
  vector<Belief> extractPredicates(const vector<Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type function
  */
  vector<Belief> extractFunctions(const vector<Belief> beliefs);

  /*
    Extract from passed vector beliefs and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGBeliefs(const vector<Belief> beliefs);

  /*
    Extract from passed vector beliefs and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGInstances(const vector<Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type predicate and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGPredicates(const vector<Belief> beliefs);

  /*
    Extract from passed vector just beliefs of type function and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGFunctions(const vector<Belief> beliefs);

  /*
    Extract from passed set just beliefs of type predicate and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGInstances(const set<ManagedBelief> managed_beliefs);
  
  /*
    Extract from passed set just beliefs of type predicate and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGPredicates(const set<ManagedBelief> managed_beliefs);

  /*
    Extract from passed set just beliefs of type function and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGFunctions(const set<ManagedBelief> managed_beliefs);

  /*
            given array of ManagedCondition, desire name, desire priority, desire deadline
            use it to build a ManagedDesire putting as value the conditions
            NOTE: for now just work around with condition(s) containing PREDICATE type as values,
            if @conditions do not contain any Belief with PREDICATE type, returns std::nullopt
  */      
  std::optional<ManagedDesire> conditionsToMGDesire(const vector<ManagedCondition>& conditions, 
                  const string& desireName, const float& desirePriority, const float& desireDeadline);
  
}  // namespace BDIFilter

#endif  // BDIFILTER__UTILS_H_