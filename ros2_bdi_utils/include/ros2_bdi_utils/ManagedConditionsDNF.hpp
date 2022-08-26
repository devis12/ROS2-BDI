#ifndef MANAGED_CONDITIONS_DNF_H_
#define MANAGED_CONDITIONS_DNF_H_

#include <string>
#include <vector>
#include <set>
#include <map>
#include <iostream>

#include "ros2_bdi_interfaces/msg/conditions_conjunction.hpp"
#include "ros2_bdi_interfaces/msg/conditions_dnf.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedConditionsConjunction.hpp"


/* Namespace for wrapper classes wrt. BDI msgs defined in ros2_bdi_interfaces::msg */
namespace BDIManaged
{

/* Wrapper class to easily manage and infer info from a ros2_bdi_interfaces::msg::ConditionsDNF instance*/
class ManagedConditionsDNF
{

    public:
        /* Constructor methods*/
        ManagedConditionsDNF();
        ManagedConditionsDNF(const ros2_bdi_interfaces::msg::ConditionsDNF& conditionsDNF);
        ManagedConditionsDNF(const std::vector<ManagedConditionsConjunction>& clauses);
        
        /* getter method for ManagedConditionsDNF instance prop -> clauses_ */
        std::vector<ManagedConditionsConjunction> getClauses() const {return clauses_;}
        
        // return true if at least one clause is satisfied against the passed belief set
        // n.b. result is true if clauses_ array is empty
        bool isSatisfied(const std::set<ManagedBelief>& mbSet);

        // convert instance to ros2_bdi_interfaces::msg::ConditionsDNF
        ros2_bdi_interfaces::msg::ConditionsDNF toConditionsDNF() const;

        // return true if the instance contains any kind of placeholder
        bool containsPlaceholders();

        // return all mg beliefs containing at least a placeholder, e.g. {x}
        std::set<ManagedBelief> getBeliefsWithPlaceholders();

        // extract assigment for all placeholders in mgconditions dnf
        std::map<std::string, std::vector<ManagedBelief>> extractAssignmentsMap(const std::set<BDIManaged::ManagedBelief>& belief_set);

        /* substitute placeholders as per assignments map and return a new ManagedConditionsDNF instance*/
        ManagedConditionsDNF applySubstitution(const std::map<std::string, std::string> assignments) const;
    private:
        /* A single clause needs to be satisfied, being this a disjunction among them */
        std::vector<ManagedConditionsConjunction> clauses_;

    };  // class ManagedConditionsDNF

    std::ostream& operator<<(std::ostream& os, const ManagedConditionsDNF& mcdnf);

    // overload `<` operator 
    bool operator<(const ManagedConditionsDNF& mcdnf1, const ManagedConditionsDNF& mcdnf2);

    // overload `==` operator 
    bool operator==(const ManagedConditionsDNF& mcdnf1, const ManagedConditionsDNF& mcdnf2);

}

#endif  // MANAGED_CONDITIONS_CONJUNCTION_H_