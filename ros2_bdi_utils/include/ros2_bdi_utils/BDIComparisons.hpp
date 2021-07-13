#ifndef BDIComparisons__UTILS_H_
#define BDIComparisons__UTILS_H_

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;

namespace BDIComparisons
{

  /*
    Check if two beliefs are equals
  */
  bool equals(const Belief b1, const Belief b2);

  /*
    Return true if the two beliefs defines the same FUNCTION_TYPE (even though they could have different values)
  */
  bool equalFunctionDefinition(const Belief b1, const Belief b2);
  
}  // namespace BDIComparisons

#endif  // BDIComparisons__UTILS_H_