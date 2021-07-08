#include <string>

#include "ros2_bdi_utils/BDIComparisons.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"
#include "ros2_bdi_utils/PDDLBDIConstants.hpp"

#define FLUENT_TYPE PDDLBDIConstants::FLUENT_TYPE
#define PREDICATE_TYPE PDDLBDIConstants::PREDICATE_TYPE

using std::string;

namespace BDIFilters
{
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
    Find position of searched belief in beliefs, return -1 if not found
  */
  int findBelief(const vector<Belief> beliefs, const Belief searched_belief)
  {
    for(int i=0; i<beliefs.size(); i++)
      if(BDIComparisons::equals(beliefs[i], searched_belief))
        return i;
    return -1;
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
  
}
