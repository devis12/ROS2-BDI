#ifndef BDIPDDLConverter__UTILS_H_
#define BDIPDDLConverter__UTILS_H_

#include <string>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "ros2_bdi_interfaces/msg/desire.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"

namespace BDIPDDLConverter
{
  /*
      Build plansys2::Instance obj from ManagedBelief
  */
  plansys2::Instance buildInstance(const BDIManaged::ManagedBelief& mb);
  
  /*
      Build plansys2::Predicate obj from ManagedBelief
  */
  plansys2::Predicate buildPredicate(const BDIManaged::ManagedBelief& mb);

  /*
      Build plansys2::Function obj from ManagedBelief
  */
  plansys2::Function buildFunction(const BDIManaged::ManagedBelief& mb);

  /*
    Convert Desire into PDDL Goal
  */
  std::string desireToGoal(const ros2_bdi_interfaces::msg::Desire& desire);
  
};  // namespace BDIPDDLConverter

#endif  // BDIPDDLConverter__UTILS_H_