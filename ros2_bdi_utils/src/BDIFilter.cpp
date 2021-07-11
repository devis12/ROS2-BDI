#include <string>

#include "ros2_bdi_utils/BDIFilter.hpp"
#include "ros2_bdi_utils/PDDLBDIConstants.hpp"

using std::string;

#define FLUENT_TYPE PDDLBDIConstants::FLUENT_TYPE
#define PREDICATE_TYPE PDDLBDIConstants::PREDICATE_TYPE

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
    Extract from passed vector beliefs and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGBeliefs(const vector<Belief> beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(Belief b : beliefs)
      if(b.type == PREDICATE_TYPE || b.type == FLUENT_TYPE)
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
      if(b.type == PREDICATE_TYPE)
          extracted.insert(ManagedBelief{b});
    return extracted;
  }

  /*
    Extract from passed vector just beliefs of type fluent and put them into a set of ManagedBelief objects
  */
  set<ManagedBelief> extractMGFluents(const vector<Belief> beliefs)
  {
    set<ManagedBelief> extracted = set<ManagedBelief>();
    for(Belief b : beliefs)
      if(b.type == FLUENT_TYPE)
          extracted.insert(ManagedBelief{b});
    return extracted;
  }

      /*
    Extract from passed set just beliefs of type predicate and put them into a set of ManagedBelief objects
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
    Extract from passed set just beliefs of type fluent and put them into a set of ManagedBelief objects
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
