#ifndef MANAGED_CONDITIONS_CONJUNCTION_H_
#define MANAGED_CONDITIONS_CONJUNCTION_H_

#include <string>
#include <vector>
#include <set>
#include <iostream>

#include "ros2_bdi_interfaces/msg/condition.hpp"
#include "ros2_bdi_interfaces/msg/conditions_conjunction.hpp"
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"

using std::string;
using std::vector;
using std::set;
using ros2_bdi_interfaces::msg::Condition;
using ros2_bdi_interfaces::msg::ConditionsConjunction;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;

class ManagedConditionsConjunction
{

    public:
        ManagedConditionsConjunction();
        ManagedConditionsConjunction(const ConditionsConjunction& conditionsConjunction);
        ManagedConditionsConjunction(const vector<ManagedCondition>& literals);
        
        vector<ManagedCondition> getLiterals() const {return literals_;}
        // returns true if all literals are satisfied against the passed belief set
        // n.b. result is true if literals_ array is empty
        bool isSatisfied(const set<ManagedBelief>& mbSet);
        
        // return ConditionsConjunction msg generated from current ManagedConditionsConjunction object
        ConditionsConjunction toConditionsConjunction() const;
        
    private:
        vector<ManagedCondition> literals_;

};  // class ManagedConditionsConjunction

std::ostream& operator<<(std::ostream& os, const ManagedConditionsConjunction& mcc);

// overload `<` operator 
bool operator<(const ManagedConditionsConjunction& mcc1, const ManagedConditionsConjunction& mcc2);

// overload `==` operator 
bool operator==(const ManagedConditionsConjunction& mcc1, const ManagedConditionsConjunction& mcc2);

#endif  // MANAGED_CONDITIONS_CONJUNCTION_H_