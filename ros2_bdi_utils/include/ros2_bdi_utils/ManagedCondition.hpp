#ifndef MANAGED_CONDITION_H_
#define MANAGED_CONDITION_H_

#include <string>
#include <vector>
#include <set>
#include <iostream>

#include "ros2_bdi_interfaces/msg/condition.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"

/* Namespace for wrapper classes wrt. BDI msgs defined in ros2_bdi_interfaces::msg */
namespace BDIManaged
{
    
    /* Wrapper class to easily manage and infer info from a ros2_bdi_interfaces::msg::Condition instance*/
    class ManagedCondition
    {

        public:
            /* Constructor methods */
            ManagedCondition(const ManagedBelief& managedBelief, const std::string& check);
            ManagedCondition(const ros2_bdi_interfaces::msg::Condition& condition);
            
            // return true iff check is VALID && condition is verified against the belief
            bool performCheckAgainstBelief(const ManagedBelief& mb);
            // return true iff check is VALID && condition is verified against the beliefs and no belief in vector denies it
            bool performCheckAgainstBeliefs(const std::vector<ManagedBelief>& mbArray);
            // return true iff check is VALID && condition is verified against the beliefs and no belief in set denies it
            bool performCheckAgainstBeliefs(const std::set<ManagedBelief>& mbSet);

            /* getter methods for ManagedCondition instance prop -> literals_ */
            ManagedBelief getMGBelief() const {return condition_to_check_;};
            std::string getCheck() const {return check_;};

            // convert instance to ros2_bdi_interfaces::msg::Condition msg
            ros2_bdi_interfaces::msg::Condition toCondition() const;

            /*
                return true iff FOR EVERY CONDITION in ManagedCondition vector 
                check is VALID && condition is verified against the beliefs and no belief in set denies it
                n.b. result is true if @mcArray is empty
            */
            static bool verifyAllManagedConditions(const std::vector<ManagedCondition>& mcArray, const std::set<ManagedBelief>& mbSet);

            /* 
                given array of Condition msg, convert it into an array of ManagedCondition
            */
            static std::vector<ManagedCondition> buildArrayMGCondition(const std::vector<ros2_bdi_interfaces::msg::Condition>& conditions);
            
        private:
            // return true iff condition to be checked is valid (e.g. cannot check smaller than for belief of type instance or predicate)
            bool validCheckRequest() const;
            // return true iff check_ is a valid check string property for an Instance type Belief
            bool isCheckStringForInstance() const;
            // return true iff check_ is a valid check string property for a Predicate type Belief
            bool isCheckStringForPredicate() const;
            // return true iff check_ is a valid check string property for a Fluent type Belief
            bool isCheckStringForFluent() const;

            /*  Belief that needs to be checked, available checks differs based on the belief type */
            ManagedBelief condition_to_check_;
            /*  Check to be performed (consult ros2_bdi_interfaces::msg::Condition msg for info)*/
            std::string check_;

    };  // class ManagedCondition

    std::ostream& operator<<(std::ostream& os, const ManagedCondition& mc);

    // overload `<` operator 
    bool operator<(const ManagedCondition& mc1, const ManagedCondition& mc2);

    // overload `==` operator 
    bool operator==(const ManagedCondition& mc1, const ManagedCondition& mc2);
}
#endif  // MANAGED_CONDITION_H_