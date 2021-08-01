#include "ros2_bdi_utils/ManagedCondition.hpp"

ManagedCondition::ManagedCondition(const ManagedBelief& managedBelief, const string& check):
    condition_to_check_(managedBelief),
    check_(check)
    {}

ManagedCondition::ManagedCondition(const Condition& condition):
    condition_to_check_(ManagedBelief{condition.condition_to_check}),
    check_(condition.check)
    {}

bool ManagedCondition::performCheckAgainstBelief(const ManagedBelief& mb)
{
    if(!validCheckRequest())//check request not valid
        return false;
    
    Condition c = Condition();

    if(condition_to_check_.pddlType() == Belief().INSTANCE_TYPE)
        return condition_to_check_ == mb;

    else if (condition_to_check_.pddlType() == Belief().PREDICATE_TYPE)
        //true if (same predicate and TRUE CHECK requested) or (diff predicate and FALSE CHECK requested)
        return (check_ == c.TRUE_CHECK)? condition_to_check_ == mb : !(condition_to_check_ == mb);

    else if (condition_to_check_.pddlType() == Belief().FUNCTION_TYPE && condition_to_check_ == mb)//has to be the same fluent
    {
        //now check the value wrt the given check request
        if(check_ == c.SMALLER_CHECK)
            return mb.getValue() < condition_to_check_.getValue();
        else if(check_ == c.SMALLER_OR_EQUALS_CHECK)
            return mb.getValue() <= condition_to_check_.getValue();
        else if(check_ == c.EQUALS_CHECK)
            return condition_to_check_.getValue() == mb.getValue();
        else if(check_ == c.GREATER_OR_EQUALS_CHECK)
            return mb.getValue() >= condition_to_check_.getValue();
        else if(check_ == c.GREATER_CHECK)
            return mb.getValue() > condition_to_check_.getValue();
    }

    return false;//different fluent or some other error(s), hence give false
}
bool ManagedCondition::performCheckAgainstBeliefs(const vector<ManagedBelief>& mbArray)
{
    if(!validCheckRequest())
        return false;
    for(ManagedBelief mb : mbArray)
        if(performCheckAgainstBelief(mb))
        {
            std::cout << "HEYHEYHEY" << check_ << " verified against " << mb << std::endl; 
            return true;
        }
    return false;
}

bool ManagedCondition::performCheckAgainstBeliefs(const set<ManagedBelief>& mbSet)
{
    if(!validCheckRequest())
        return false;
    for(ManagedBelief mb : mbSet)
        if(performCheckAgainstBelief(mb))
            return true;
    return false;
}


Condition ManagedCondition::toCondition() const
{
    Condition conditionMsg = Condition();
    conditionMsg.condition_to_check = condition_to_check_.toBelief();
    conditionMsg.check = check_;
    return conditionMsg;
}

bool ManagedCondition::validCheckRequest() const
{
    bool validCheckInstance = condition_to_check_.pddlType() == Belief().INSTANCE_TYPE && isCheckStringForInstance();
    bool validCheckPredicate = condition_to_check_.pddlType() == Belief().PREDICATE_TYPE && isCheckStringForPredicate();
    bool validCheckFluent = condition_to_check_.pddlType() == Belief().FUNCTION_TYPE && isCheckStringForFluent();

    return validCheckInstance || validCheckPredicate || validCheckFluent;
}


bool ManagedCondition::isCheckStringForInstance() const
{
    Condition c = Condition();
    return check_ == c.EXISTS_CHECK;
}

bool ManagedCondition::isCheckStringForPredicate() const
{
    Condition c = Condition();
    return check_ == c.TRUE_CHECK || check_ == c.FALSE_CHECK;
}

bool ManagedCondition::isCheckStringForFluent() const
{
    Condition c = Condition();
    return check_ == c.EQUALS_CHECK || check_ == c.GREATER_CHECK || check_ == c.SMALLER_CHECK ||
        check_ == c.SMALLER_OR_EQUALS_CHECK || check_ == c.GREATER_OR_EQUALS_CHECK;
}

bool ManagedCondition::verifyAllManagedConditions(
        const vector<ManagedCondition>& mcArray, const set<ManagedBelief>& mbSet)
{
    for(ManagedCondition mc : mcArray)
        if(!mc.performCheckAgainstBeliefs(mbSet))//one condition not valid and/or not verified
            return false;
            
    return true;
}

 vector<ManagedCondition> ManagedCondition::buildArrayMGCondition(const vector<Condition>& conditions)
 {
    vector<ManagedCondition> result;
    for(Condition c : conditions)
        result.push_back(ManagedCondition{c});
    return result;
 }

std::ostream& operator<<(std::ostream& os, const ManagedCondition& mc)
{
    string check_string = mc.getCheck();
    Condition c = Condition();
    if(check_string == c.EXISTS_CHECK)
        check_string = "EXISTS";
    else if(check_string == c.TRUE_CHECK)
        check_string = "TRUE";
    else if(check_string == c.FALSE_CHECK)
        check_string = "FALSE";
    else if(check_string == c.SMALLER_CHECK)
        check_string = "SMALLER";
    else if(check_string == c.SMALLER_OR_EQUALS_CHECK)
        check_string = "SMALLER OR EQUALS";
    else if(check_string == c.EQUALS_CHECK)
        check_string = "EQUALS";
    else if(check_string == c.GREATER_OR_EQUALS_CHECK)
        check_string = "GREATER OR EQUALS";
    else if(check_string == c.GREATER_CHECK)
        check_string = "GREATER";

    os << "CHECK TO BE PERFORMED:" << check_string  << " \n" << mc.getMGBelief();
    return os;
}

// overload `<` operator 
bool operator<(const ManagedCondition& mc1, const ManagedCondition& mc2)
{
    return mc1.getMGBelief() < mc2.getMGBelief() && mc1.getCheck() < mc2.getCheck();
}

// overload `==` operator 
bool operator==(const ManagedCondition& mc1, const ManagedCondition& mc2)
{
    return mc1.getMGBelief() == mc2.getMGBelief() && mc1.getCheck() == mc2.getCheck();
}   