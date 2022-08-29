#ifndef MANAGED_REACTIVE_RULE_H_
#define MANAGED_REACTIVE_RULE_H_

#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>
#include <iostream>

#include "ros2_bdi_interfaces/msg/desire.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedConditionsConjunction.hpp"
#include "ros2_bdi_utils/ManagedConditionsDNF.hpp"

//allowed operation in rules
typedef enum {ADD, DEL} ReactiveOp;

//Rule type for belief op.
typedef std::pair<ReactiveOp, BDIManaged::ManagedBelief> MGBeliefOp;
//Rule type for desire op.
typedef std::pair<ReactiveOp, BDIManaged::ManagedDesire> MGDesireOp;

/* Namespace for wrapper classes wrt. BDI msgs defined in ros2_bdi_interfaces::msg and composition of them */
namespace BDIManaged
{

    /* Wrapper class to easily manage and infer info from Reactive Rule instance (Condition -> Bset/Dset upd. rules to apply)*/
    class ManagedReactiveRule
    {

        public:
            //constructor methods
            ManagedReactiveRule( const uint8_t, const BDIManaged::ManagedConditionsDNF&,
                const std::set<MGBeliefOp>&, const std::set<MGDesireOp>&);

            // getter methods
            uint8_t getId() const{return ai_id_;}
            BDIManaged::ManagedConditionsDNF getMGCondition() const{return dnf_condition_;}
            std::set<MGBeliefOp> getBeliefRules() const{return belief_rules_;}
            std::set<MGDesireOp> getDesireRules() const{return desire_rules_;}

            static ManagedReactiveRule applySubstitution(const ManagedReactiveRule& baseline_reactive_rule, const std::map<std::string, std::string> assignments);
        private:

            uint8_t ai_id_;//auto generated in init -> used to put all the rules in a set easily
            BDIManaged::ManagedConditionsDNF dnf_condition_;//condition that must be true wrt. the current belief set to apply the rule
            std::set<MGBeliefOp> belief_rules_;//rules to apply to the belief set 
            std::set<MGDesireOp> desire_rules_;//rules to apply to the desire set

    };  // class ManagedReactiveRule

    std::ostream& operator<<(std::ostream& os, const ManagedReactiveRule& mr);

    // overload `<` operator 
    bool operator<(const ManagedReactiveRule& mr1, const ManagedReactiveRule& mr2);
}

#endif  // MANAGED_REACTIVE_RULE_H_