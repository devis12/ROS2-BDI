#include <string>

#include "ros2_bdi_utils/BDIFilter.hpp"

using std::string;

namespace BDIFilter
{

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
    Extract from passed set of ManagedDesire objects a DesireSet msg
  */
  DesireSet extractDesireSetMsg(const set<ManagedDesire> managed_desires)
  {
    DesireSet dset_msg = DesireSet();
    vector<Desire> desires_vector = vector<Desire>();
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
        if(b.type == Belief().INSTANCE_TYPE)
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
        if(b.type == Belief().PREDICATE_TYPE)
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
        if(b.type == Belief().FUNCTION_TYPE)
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
      if(b.type == Belief().INSTANCE_TYPE || b.type == Belief().PREDICATE_TYPE || b.type == Belief().FUNCTION_TYPE)
          extracted.insert(ManagedBelief{b});
    return extracted;
  }

  /*
    Extract from passed vector beliefs and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGInstances(const vector<Belief> beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(Belief b : beliefs)
      if(b.type == Belief().INSTANCE_TYPE)
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
      if(b.type == Belief().PREDICATE_TYPE)
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
      if(b.type == Belief().FUNCTION_TYPE)
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
      if(mb.type_ == Belief().INSTANCE_TYPE)
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
      if(mb.type_ == Belief().PREDICATE_TYPE)
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
      if(mb.type_ == Belief().FUNCTION_TYPE)
          extracted.insert(mb);
    return extracted;
  }

}
