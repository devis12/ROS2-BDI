#include "ros2_bdi_utils/ManagedReactiveRule.hpp"

using BDIManaged::ManagedReactiveRule;

ManagedReactiveRule::ManagedReactiveRule( 
    const uint8_t id, const BDIManaged::ManagedConditionsDNF& condition,
    const std::set<MGBeliefOp>& bset_rules, const std::set<MGDesireOp>& dset_rules) : 
    ai_id_(id), dnf_condition_(condition),
    belief_rules_(bset_rules), desire_rules_(dset_rules)
{
    //
}

std::ostream& BDIManaged::operator<<(std::ostream& os, const ManagedReactiveRule& mr)
{

    os << "\nRULE_ID" << mr.getId();
    os << "\nCondition to be true: " << mr.getMGCondition();
    os << "\nBelief set rules to apply:\n";
    for(auto bset_rule : mr.getBeliefRules())
        os << "\t" << ((bset_rule.first == ReactiveOp::ADD)? "ADD" : "DEL") << bset_rule.second;
    for(auto dset_rule : mr.getDesireRules())
        os << "\t" << ((dset_rule.first == ReactiveOp::ADD)? "ADD" : "DEL") << dset_rule.second;
    return os;
}

bool BDIManaged::operator<(const ManagedReactiveRule& mr1, const ManagedReactiveRule& mr2)
{
    return mr1.getId() < mr2.getId();
}
