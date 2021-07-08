#include <string>

#include "ros2_bdi_utils/BDIComparisons.hpp"
#include "ros2_bdi_utils/PDDLBDIConstants.hpp"

#define FLUENT_TYPE PDDLBDIConstants::FLUENT_TYPE
#define PREDICATE_TYPE PDDLBDIConstants::PREDICATE_TYPE

using std::string;

namespace BDIComparisons
{
   /*
    Check if two beliefs are equals
  */
  bool equals(const Belief b1, const Belief b2)
  {    
    //check for different types or type fluent && different values
    if(b1.type != b2.type || (b1.type == FLUENT_TYPE && b1.value != b2.value))
        return false;

    //check for different name or different num of params
    if(b1.name != b2.name || b1.params.size() != b2.params.size())
        return false;

    //check equals param by param (at this point you know the two arrays are the same size)
    for(int i=0;i<b1.params.size();i++)
        if(b1.params[i] != b2.params[i])
            return false;

    //otherwise equals
    return true;
  }

  /*
    Return true if the two beliefs defines the same fluent (even though they could have different values)
  */
  bool equalFluentDefinition(const Belief b1, const Belief b2)
  {
    //check for different types from FLUENT
    if(b1.type != FLUENT_TYPE || b2.type != FLUENT_TYPE)
        return false;
    
     //check for different name or different num of params
    if(b1.name != b2.name || b1.params.size() != b2.params.size())
        return false;

    //check equals param by param (at this point you know the two arrays are the same size)
    for(int i=0;i<b1.params.size();i++)
        if(b1.params[i] != b2.params[i])
            return false;

    return true;
  }
}
