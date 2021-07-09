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
      beliefs.push_back(convertPDDLPredicate(p));
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

    /*
    Extract from passed set of ManagedBelief objects a BeliefSet msg
  */
  BeliefSet extractBeliefSetMsg(const set<ManagedBelief> managed_beliefs)
  {
    BeliefSet bset_msg = BeliefSet();
    vector<Belief> beliefs_vector = vector<Belief>();
    for(ManagedBelief mb : managed_beliefs)
      beliefs_vector.push_back(mb.toBelief());
    bset_msg.value = beliefs_vector;
    return bset_msg; 
  }

    /*
    Extract from passed vector just beliefs of type predicate
  */
  vector<Belief> extractPredicates(const vector<Belief> beliefs)
  {
    vector<Belief> extracted = vector<Belief>();
    for(Belief b : beliefs)
        if(b.type == PREDICATE_TYPE)
            extracted.push_back(b);
    return extracted;
  } 

  /*
    Extract from passed vector just beliefs of type fluent
  */
  vector<Belief> extractFluents(const vector<Belief> beliefs)
  {
    vector<Belief> extracted = vector<Belief>();
    for(Belief b : beliefs)
        if(b.type == FLUENT_TYPE)
            extracted.push_back(b);
    return extracted;
  }

    /*
    Extract from passed vector just beliefs of type predicate and put it into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGPredicates(const vector<Belief> beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(Belief b : beliefs)
      if(b.type == PREDICATE_TYPE)
          extracted.insert(b);
    return extracted;
  }

  /*
    Extract from passed vector just beliefs of type fluent and put it into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGFluents(const vector<Belief> beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(Belief b : beliefs)
      if(b.type == FLUENT_TYPE)
          extracted.insert(b);
    return extracted;
  }

      /*
    Extract from passed set just beliefs of type predicate and put it into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGPredicates(const set<ManagedBelief> managed_beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(ManagedBelief mb : managed_beliefs)
      if(mb.type_ == PREDICATE_TYPE)
          extracted.insert(mb);
    return extracted;
  }

  /*
    Extract from passed set just beliefs of type fluent and put it into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGFluents(const set<ManagedBelief> managed_beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(ManagedBelief mb : managed_beliefs)
      if(mb.type_ == FLUENT_TYPE)
          extracted.insert(mb);
    return extracted;
  }

}
