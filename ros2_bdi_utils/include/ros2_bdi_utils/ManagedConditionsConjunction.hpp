#ifndef MANAGED_CONDITIONS_CONJUNCTION_H_
#define MANAGED_CONDITIONS_CONJUNCTION_H_

#include <vector>
#include <set>
#include <iostream>

#include "ros2_bdi_interfaces/msg/condition.hpp"
#include "ros2_bdi_interfaces/msg/conditions_conjunction.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"

namespace BDIManaged
{

    class ManagedConditionsConjunction
    {

        public:
            ManagedConditionsConjunction();
            ManagedConditionsConjunction(const ros2_bdi_interfaces::msg::ConditionsConjunction& conditionsConjunction);
            ManagedConditionsConjunction(const std::vector<ManagedCondition>& literals);
            
            std::vector<ManagedCondition> getLiterals() const {return literals_;}
            // returns true if all literals are satisfied against the passed belief set
            // n.b. result is true if literals_ array is empty
            bool isSatisfied(const std::set<ManagedBelief>& mbSet);
            
            // convert to ConditionsConjunction msg generated from current ManagedConditionsConjunction object
            ros2_bdi_interfaces::msg::ConditionsConjunction toConditionsConjunction() const;

        private:
            std::vector<ManagedCondition> literals_;

    };  // class ManagedConditionsConjunction

    std::ostream& operator<<(std::ostream& os, const ManagedConditionsConjunction& mcc);

    // overload `<` operator 
    bool operator<(const ManagedConditionsConjunction& mcc1, const ManagedConditionsConjunction& mcc2);

    // overload `==` operator 
    bool operator==(const ManagedConditionsConjunction& mcc1, const ManagedConditionsConjunction& mcc2);

}

#endif  // MANAGED_CONDITIONS_CONJUNCTION_H_