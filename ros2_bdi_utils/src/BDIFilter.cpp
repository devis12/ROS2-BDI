#include "ros2_bdi_utils/BDIFilter.hpp"

#include <string>

#include "ros2_bdi_interfaces/msg/condition.hpp"

using std::string;
using std::vector;
using std::set;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::DesireSet;
using ros2_bdi_interfaces::msg::Condition;

using BDIManaged::ManagedType;
using BDIManaged::ManagedBelief;
using BDIManaged::ManagedDesire;
using BDIManaged::ManagedCondition;
using BDIManaged::ManagedConditionsConjunction;
using BDIManaged::ManagedConditionsDNF;

namespace BDIFilter
{

  /*
    Extract from passed set of ManagedBelief objects a BeliefSet msg
  */
  BeliefSet extractBeliefSetMsg(const set<ManagedBelief> managed_beliefs)
  {
    BeliefSet bset_msg = BeliefSet();
    vector<Belief> beliefs_vector = vector<Belief>();
    
    // iterate over the ManagedBelief set and convert every item to a Belief msg to be pushed 
    // into the array value for the BeliefSet obj
    for(ManagedBelief mb : managed_beliefs)
      beliefs_vector.push_back(mb.toBelief());

    bset_msg.value = beliefs_vector;
    return bset_msg; 
  }

  /*
    Extract from passed set of ManagedDesire objects a DesireSet msg
  */
  DesireSet extractDesireSetMsg(const set<ManagedDesire> managed_desires)
  {
    DesireSet dset_msg = DesireSet();
    vector<Desire> desires_vector = vector<Desire>();
        
    // iterate over the ManagedDesire set and convert every item to a Desire msg to be pushed 
    // into the array value for the DesireSet obj
    for(ManagedDesire md : managed_desires)
      desires_vector.push_back(md.toDesire());

    dset_msg.value = desires_vector;
    return dset_msg; 
  }

  /*
    Extract from passed vector just beliefs of type instance
  */
  vector<Belief> extractInstances(const vector<Belief> beliefs)
  {
    vector<Belief> extracted = vector<Belief>();
    for(Belief b : beliefs)
        if(b.pddl_type == Belief().INSTANCE_TYPE)
            extracted.push_back(b);
    return extracted;
  }

  /*
    Extract from passed vector just beliefs of type predicate
  */
  vector<Belief> extractPredicates(const vector<Belief> beliefs)
  {
    vector<Belief> extracted = vector<Belief>();
    for(Belief b : beliefs)
        if(b.pddl_type == Belief().PREDICATE_TYPE)
            extracted.push_back(b);
    return extracted;
  } 

  /*
    Extract from passed vector just beliefs of type function
  */
  vector<Belief> extractFunctions(const vector<Belief> beliefs)
  {
    vector<Belief> extracted = vector<Belief>();
    for(Belief b : beliefs)
        if(b.pddl_type == Belief().FUNCTION_TYPE)
            extracted.push_back(b);
    return extracted;
  }

   /*
    Extract from passed vector beliefs and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGBeliefs(const vector<Belief> beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(Belief b : beliefs)
      if(b.pddl_type == Belief().INSTANCE_TYPE || b.pddl_type == Belief().PREDICATE_TYPE || b.pddl_type == Belief().FUNCTION_TYPE)
          extracted.insert(ManagedBelief{b});
    return extracted;
  }

  /*
    Extract from passed vector desires and put them into a set of ManagedDesire objects
  */
  set<ManagedDesire> extractMGDesires(const vector<Desire> desires)
  {
    set<ManagedDesire> extracted = set<ManagedDesire>();
    for(Desire d : desires)
      extracted.insert(ManagedDesire{d});
    return extracted;
  }

  /*
    Extract from passed vector beliefs and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGInstances(const vector<Belief> beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(Belief b : beliefs)
      if(b.pddl_type == Belief().INSTANCE_TYPE)
          extracted.insert(ManagedBelief{b});
    return extracted;
  }
   
  /*
    Extract from passed vector just beliefs of type predicate and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGPredicates(const vector<Belief> beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(Belief b : beliefs)
      if(b.pddl_type == Belief().PREDICATE_TYPE)
          extracted.insert(ManagedBelief{b});
    return extracted;
  }

  /*
    Extract from passed vector just beliefs of type function and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGFunctions(const vector<Belief> beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(Belief b : beliefs)
      if(b.pddl_type == Belief().FUNCTION_TYPE)
          extracted.insert(ManagedBelief{b});
    return extracted;
  }
  
  /*
    Extract from passed set just beliefs of type predicate and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGInstances(const set<ManagedBelief> managed_beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(ManagedBelief mb : managed_beliefs)
      if(mb.pddlType() == Belief().INSTANCE_TYPE)
          extracted.insert(mb);
    return extracted;
  }

  /*
    Extract from passed set just beliefs of type predicate and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGPredicates(const set<ManagedBelief> managed_beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(ManagedBelief mb : managed_beliefs)
      if(mb.pddlType() == Belief().PREDICATE_TYPE)
          extracted.insert(mb);
    return extracted;
  }

  /*
    Extract from passed set just beliefs of type function and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGFunctions(const set<ManagedBelief> managed_beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(ManagedBelief mb : managed_beliefs)
      if(mb.pddlType() == Belief().FUNCTION_TYPE)
          extracted.insert(mb);
    return extracted;
  }

  /*
    Given array of ManagedCondition, desire base name (added a counter as suffix to distinguish them among each other),
    desire priority, desire deadline use it to build a ManagedDesire putting as value the conditions
    NOTE: for now just work around with condition(s) containing PREDICATE type as values,
    if @conditions do not contain any Belief with PREDICATE type, returns empty array
  */    
  vector<ManagedDesire> conditionsToMGDesire(const ManagedConditionsDNF& conditionsDNF, 
                            const string& desireBaseName, 
                            const float& desirePriority, const float& desireDeadline)
  {
      vector<ManagedDesire> extractedDesires;
      int counter = 0;

      // Iterate over all clauses: each one of them represents a desire's value
      for(ManagedConditionsConjunction mcc : conditionsDNF.getClauses())
      {
        counter++;

        string desireName = desireBaseName + std::to_string(counter);
        vector<ManagedBelief> desireValue;
        for(ManagedCondition mc : mcc.getLiterals())
        {
          ManagedBelief mb = mc.getMGBelief();

          // for now desire can support just value with PREDICATE type Beliefs on TRUE checks
          // [TODO] update when upgrading to new PDDL planners
          if(mc.getCheck() == Condition().TRUE_CHECK && mb.pddlType() == Belief().PREDICATE_TYPE)
            desireValue.push_back(mb);
        }

        if(desireValue.size() > 0)
          extractedDesires.push_back(ManagedDesire{desireName, desireValue, desirePriority, desireDeadline});
      }

      return extractedDesires;
      
  }

  set<ManagedBelief> filterMGBeliefInstances(const set<ManagedBelief>& belief_set, const ManagedType& type)
  {
    set<ManagedBelief> belief_set_filtered;
    // std::cout << "Looking for type " << type.name;
    // if(type.sub_types.has_value())
    //   for(string s : type.sub_types.value())
    //     std::cout << " or subtype: " << s << ",";
    // std::cout << std::flush << std::endl;
    for(ManagedBelief mb : belief_set)
      if(mb.pddlType() == Belief().INSTANCE_TYPE)
        if(type.name == "" || type.name == mb.type().name)
          belief_set_filtered.insert(mb);
        else if(type.sub_types.has_value() && 
          std::find(type.sub_types.value().begin(), type.sub_types.value().end(), mb.type().name) != std::end(type.sub_types.value()))
          belief_set_filtered.insert(mb);
        
    return belief_set_filtered;
  }
  

}
