#ifndef BDIFilters__UTILS_H_
#define BDIFilters__UTILS_H_

#include <vector>
#include <set>
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"

using std::vector;
using std::set;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;

namespace BDIFilter
{

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

  /*
    Extract from passed set of ManagedBelief objects a BeliefSet msg
  */
  BeliefSet extractBeliefSetMsg(const set<ManagedBelief> managed_beliefs);
  
  /*
    Find position of searched belief in beliefs, return -1 if not found
  */
  int findBelief(const vector<Belief> beliefs, const Belief searched_belief);
  
}  // namespace BDIFilters

#endif  // BDIFilters__UTILS_H_