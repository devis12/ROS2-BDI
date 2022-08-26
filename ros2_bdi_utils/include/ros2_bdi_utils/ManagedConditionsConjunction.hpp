#ifndef MANAGED_CONDITIONS_CONJUNCTION_H_
#define MANAGED_CONDITIONS_CONJUNCTION_H_

#include <vector>
#include <set>
#include <iostream>

#include "ros2_bdi_interfaces/msg/condition.hpp"
#include "ros2_bdi_interfaces/msg/conditions_conjunction.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"

/* Namespace for wrapper classes wrt. BDI msgs defined in ros2_bdi_interfaces::msg */
namespace BDIManaged
{

    /* Wrapper class to easily manage and infer info from a ros2_bdi_interfaces::msg::ConditionsConjunction instance*/
    class ManagedConditionsConjunction
    {

        public:
            /* Constructor methods*/
            ManagedConditionsConjunction();
            ManagedConditionsConjunction(const ros2_bdi_interfaces::msg::ConditionsConjunction& conditionsConjunction);
            ManagedConditionsConjunction(const std::vector<ManagedCondition>& literals);
            
            /* getter method for ManagedConditionsConjunction instance prop -> literals_ */
            std::vector<ManagedCondition> getLiterals() const {return literals_;}

            // returns true if all literals are satisfied against the passed belief set
            // n.b. result is true if literals_ array is empty
            bool isSatisfied(const std::set<ManagedBelief>& mbSet);
            
            // convert instance to ros2_bdi_interfaces::msg::ConditionsConjunction format
            ros2_bdi_interfaces::msg::ConditionsConjunction toConditionsConjunction() const;

            // return true if the instance contains any kind of placeholder
            bool containsPlaceholders();

            // return all mg beliefs containing at least a placeholder, e.g. {x}
            std::set<ManagedBelief> getBeliefsWithPlaceholders();

            /* substitute placeholders as per assignments map and return a new ManagedConditionsConjunction instance*/
            ManagedConditionsConjunction applySubstitution(const std::map<std::string, std::string> assignments) const;

        private:
            /* All literals need to be satisfied being this a conjunction among them */
            std::vector<ManagedCondition> literals_;

    };  // class ManagedConditionsConjunction

    std::ostream& operator<<(std::ostream& os, const ManagedConditionsConjunction& mcc);

    // overload `<` operator 
    bool operator<(const ManagedConditionsConjunction& mcc1, const ManagedConditionsConjunction& mcc2);

    // overload `==` operator 
    bool operator==(const ManagedConditionsConjunction& mcc1, const ManagedConditionsConjunction& mcc2);

}

#endif  // MANAGED_CONDITIONS_CONJUNCTION_H_