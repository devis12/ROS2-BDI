#ifndef MANAGED_CONDITION_H_
#define MANAGED_CONDITION_H_

#include <string>
#include <vector>
#include <set>
#include <iostream>

#include "ros2_bdi_interfaces/msg/condition.hpp"
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"

using std::string;
using std::vector;
using std::set;
using ros2_bdi_interfaces::msg::Condition;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;

class ManagedCondition
{

    public:
        ManagedCondition(const ManagedBelief& managedBelief, const string& check);
        ManagedCondition(const Condition& condition);
        
        // return true iff check is VALID && condition is verified against the belief
        bool performCheckAgainstBelief(const ManagedBelief& mb);
        // return true iff check is VALID && condition is verified against the beliefs and no belief in vector denies it
        bool performCheckAgainstBeliefs(const vector<ManagedBelief>& mbArray);
        // return true iff check is VALID && condition is verified against the beliefs and no belief in set denies it
        bool performCheckAgainstBeliefs(const set<ManagedBelief>& mbSet);

        ManagedBelief getMGBelief() const {return condition_to_check_;};
        string getCheck() const {return check_;};

        // convert to condition msg
        Condition toCondition() const;

        // return true iff FOR EVERY CONDITION in ManagedCondition vector 
        // check is VALID && condition is verified against the beliefs and no belief in set denies it
        // n.b. result is true if @mcArray is empty
        static bool verifyAllManagedConditions(const vector<ManagedCondition>& mcArray, const set<ManagedBelief>& mbSet);

        static vector<ManagedCondition> buildArrayMGCondition(const vector<Condition>& conditions);
        
    private:
        // return true iff condition to be checked is valid (e.g. cannot check smaller than for belief of type instance or predicate)
        bool validCheckRequest() const;
        // return true iff check_ is a valid check string property for an Instance type Belief
        bool isCheckStringForInstance() const;
        // return true iff check_ is a valid check string property for a Predicate type Belief
        bool isCheckStringForPredicate() const;
        // return true iff check_ is a valid check string property for a Fluent type Belief
        bool isCheckStringForFluent() const;

        ManagedBelief condition_to_check_;
        string check_;

};  // class ManagedCondition

std::ostream& operator<<(std::ostream& os, const ManagedCondition& mc);

// overload `<` operator 
bool operator<(const ManagedCondition& mc1, const ManagedCondition& mc2);

// overload `==` operator 
bool operator==(const ManagedCondition& mc1, const ManagedCondition& mc2);

#endif  // MANAGED_CONDITION_H_