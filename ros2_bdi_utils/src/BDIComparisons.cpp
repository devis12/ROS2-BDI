#include <string>

#include "ros2_bdi_utils/BDIComparisons.hpp"

using std::string;

namespace BDIComparisons
{
   /*
    Check if two beliefs are equals
  */
  bool equals(const Belief b1, const Belief b2)
  {    
    //check for different types or type function && different values
    if(b1.type != b2.type || (b1.type == Belief().FUNCTION_TYPE && b1.value != b2.value))
        return false;

    //if instance type check for name & params[0]
    if(b1.type == Belief().INSTANCE_TYPE)
      return b1.name == b2.name && b1.params.size() == 1 && b2.params.size() == 1 && 
            b1.params[0] == b2.params[0];

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
    Return true if the two beliefs defines the same function (even though they could have different values)
  */
  bool equalFunctionDefinition(const Belief b1, const Belief b2)
  {
    //check for different types from function
    if(b1.type != Belief().FUNCTION_TYPE || b2.type != Belief().FUNCTION_TYPE)
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
