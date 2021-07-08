#include <string>

#include "ros2_bdi_utils/PDDLBDIConverter.hpp"
#include "ros2_bdi_utils/PDDLBDIConstants.hpp"

using std::string;

#define FLUENT_TYPE PDDLBDIConstants::FLUENT_TYPE
#define PREDICATE_TYPE PDDLBDIConstants::PREDICATE_TYPE

namespace PDDLBDIConverter
{
   /*
    Convert PlanSys2 PDDL Predicate to ROS2-BDI Belief
  */
  Belief convertPDDLPredicate(const Predicate predicate)
  {
    Belief b = Belief();
    
    b.name = predicate.name;
    b.type = PREDICATE_TYPE;

    vector<string> params = vector<string>();
    for(auto p : predicate.parameters)
        params.push_back(p.name);
    b.params = params;
    
    b.value = 0.0f;// has NO meaning in Function
    
    return b;
  }


  /*
    Convert PlanSys2 PDDL Predicates to ROS2-BDI Beliefs
  */
  vector<Belief> convertPDDLPredicates(const vector<Predicate> predicates){
    vector<Belief> beliefs = vector<Belief>();
    for(auto p : predicates)
      beliefs.push_back(convertPDDLFunction(p));
    return beliefs;
  }


    /*
    Convert PlanSys2 PDDL Function to ROS2-BDI Belief
  */
  Belief convertPDDLFunction(const Function function)
  {
    Belief b = Belief();
    
    b.name = function.name;
    b.type = FLUENT_TYPE;

    vector<string> params = vector<string>();
    for(auto p : function.parameters)
        params.push_back(p.name);
    b.params = params;
    
    b.value = function.value;
    
    return b;
  }

    /*
    Convert PlanSys2 PDDL Functions to ROS2-BDI Beliefs
  */
  vector<Belief> convertPDDLFunctions(const vector<Function> functions){
    vector<Belief> beliefs = vector<Belief>();
    for(auto f : functions)
      beliefs.push_back(convertPDDLFunction(f));
    return beliefs;
  }

}
