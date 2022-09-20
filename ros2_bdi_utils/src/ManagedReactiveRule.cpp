#include "ros2_bdi_utils/ManagedReactiveRule.hpp"

using BDIManaged::ManagedReactiveRule;

ManagedReactiveRule::ManagedReactiveRule( 
    const uint16_t id, const BDIManaged::ManagedConditionsDNF& condition,
    const std::set<MGBeliefOp>& bset_rules, const std::set<MGDesireOp>& dset_rules) : 
    ai_id_(id), dnf_condition_(condition),
    belief_rules_(bset_rules), desire_rules_(dset_rules)
{
    //
}

ManagedReactiveRule ManagedReactiveRule::applySubstitution(const ManagedReactiveRule& baseline_reactive_rule, const std::map<std::string, std::string> assignments)
{
    ManagedConditionsDNF conditions_dnf = baseline_reactive_rule.getMGCondition().applySubstitution(assignments);
    
    std::set<MGBeliefOp> baseline_beliefs_op = baseline_reactive_rule.getBeliefRules();
    std::set<MGBeliefOp> new_beliefs_op;
    for(auto it = baseline_beliefs_op.begin(); it != baseline_beliefs_op.end(); ++it)
        new_beliefs_op.insert(std::make_pair(it->first, it->second.applySubstitution(assignments)));
        
    std::set<MGDesireOp> baseline_desires_op = baseline_reactive_rule.getDesireRules();
    std::set<MGDesireOp> new_desires_op;
    for(auto it = baseline_desires_op.begin(); it != baseline_desires_op.end(); ++it)
        new_desires_op.insert(std::make_pair(it->first, it->second.applySubstitution(assignments)));

    return ManagedReactiveRule{baseline_reactive_rule.getId(), conditions_dnf, new_beliefs_op, new_desires_op};
}

std::ostream& BDIManaged::operator<<(std::ostream& os, const ManagedReactiveRule& mr)
{

    os << "\nRULE_ID" << std::to_string(mr.getId());
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
