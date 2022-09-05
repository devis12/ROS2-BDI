#include "ros2_bdi_utils/ManagedCondition.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"


using std::string;
using std::vector;
using std::map;
using std::set;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Condition;

using BDIManaged::ManagedParam;
using BDIManaged::ManagedBelief;
using BDIManaged::ManagedCondition;

/*
    returns true if the wild_string potentially containing wild characters that are meant to be replaced by a single char (wild_single_char) or multiple ones (wild_multi_char)
    mathes the text_string
    Krauss algorithm https://en.wikipedia.org/wiki/Krauss_wildcard-matching_algorithm 
*/
bool krauss_wild_pattern_match(string wild_string, string text_string, 
    const char& wild_single_char = '?', const char& wild_multi_char = '*')
{
    if(wild_string == text_string || wild_string == std::to_string(wild_multi_char))//trivial case
        return true;

    bool TRUE=true,FALSE=false;
    bool check[wild_string.length()+1][text_string.length()+1];
    check[0][0]=TRUE;
    
    for(int i=1;i<=text_string.length();i++)
       check[0][i]=FALSE;
    for(int i=1;i<=wild_string.length();i++)
        if(wild_string[i-1] == wild_multi_char)//Checking for wild characters.
            check[i][0]=check[i-1][0];
    else
        check[i][0]=FALSE;
        
    for(int i=1;i<=wild_string.length();i++)
    {
        for(int j=1;j<=text_string.length();j++)
        {
            if(wild_string[i-1] == text_string[j-1])
                check[i][j]=check[i-1][j-1];
            else if(wild_string[i-1] == wild_single_char)//Checking for wild character '?'.
                check[i][j]=check[i-1][j-1];
            else if(wild_string[i-1] == wild_multi_char)//Checking for wild character '*'.
                check[i][j]=check[i-1][j]||check[i][j-1];
                
            else
               check[i][j] =FALSE;
        }
    }

    return check[wild_string.length()][text_string.length()];
}

/*
    Returns true if the two mg beliefs are equivalent, taking into consideration wild pattern in the first
    (e.g. params={"box_*"} will be considered equivalent to params={"box_a1"} ) when comparing names and params
    (does not apply to value which are not a factor in order to consider the equivalence of two managed belief)
*/
bool wild_pattern_equal(const ManagedBelief& mgBeliefWildPattern, const ManagedBelief& mgBeliefToCheckAgainst)
{
    if(mgBeliefWildPattern.pddlType() != mgBeliefToCheckAgainst.pddlType()) // different pddl type... no reason to go further in the comparison
        return false;
        
    if(!krauss_wild_pattern_match(mgBeliefWildPattern.getName(), mgBeliefToCheckAgainst.getName())) // names do not match (considering wild pattern chars)
        return false;

    vector<ManagedParam> wild_params = mgBeliefWildPattern.getParams();
    vector<ManagedParam> text_params = mgBeliefToCheckAgainst.getParams();
    if(wild_params.size() != mgBeliefToCheckAgainst.getParams().size())// params size differ
        return false;
    
    //check equals param by param (at this point you know the two arrays are the same size)
    for(uint8_t i = 0; i < wild_params.size(); i++)
    {   
        if(!krauss_wild_pattern_match(wild_params[i].name,text_params[i].name)) //params in pos i do not match
            return false;
    }
    //otherwise equals
    return true;
}

ManagedCondition::ManagedCondition(const ManagedBelief& managedBelief, const string& check):
    condition_to_check_(managedBelief),
    check_(check)
    {}

ManagedCondition::ManagedCondition(const Condition& condition):
    condition_to_check_(ManagedBelief{condition.condition_to_check}),
    check_(condition.check)
    {}

// Clone a MG Conditions DNF
ManagedCondition ManagedCondition::clone()
{
    return ManagedCondition{condition_to_check_.clone(), string{check_}};
}

/* substitute placeholders as per assignments map and return a new ManagedCondition instance*/
ManagedCondition ManagedCondition::applySubstitution(const map<string, string> assignments) const
{
    return ManagedCondition{condition_to_check_.applySubstitution(assignments), check_};
}

bool ManagedCondition::containsPlaceholders(){
    if(condition_to_check_.pddlType() == Belief().PREDICATE_TYPE || condition_to_check_.pddlType() == Belief().FUNCTION_TYPE)   
    {
        for(ManagedParam arg : condition_to_check_.getParams())
            if(arg.isPlaceholder())
               return true;
    }
    else if(condition_to_check_.pddlType() == Belief().INSTANCE_TYPE)
    {
        string instance_name = condition_to_check_.getName();
        if(instance_name.find("{") == 0 && instance_name.find("}") == instance_name.length()-1)
            return true;
    }
    return false;// no placeholder found
}

bool ManagedCondition::performCheckAgainstBelief(const ManagedBelief& mb)
{
    if(!validCheckRequest())//check request not valid
        return false;
    
    Condition c = Condition();

    if(condition_to_check_.pddlType() == Belief().INSTANCE_TYPE)
        return wild_pattern_equal(condition_to_check_, mb);

    else if (condition_to_check_.pddlType() == Belief().PREDICATE_TYPE)
        //true if (same predicate and TRUE CHECK requested) or (diff predicate and FALSE CHECK requested)
        return (check_ == c.TRUE_CHECK)? wild_pattern_equal(condition_to_check_, mb) : !(wild_pattern_equal(condition_to_check_, mb));

    else if (condition_to_check_.pddlType() == Belief().FUNCTION_TYPE && wild_pattern_equal(condition_to_check_, mb))//has to be the same fluent
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
    Condition c = Condition();
    uint8_t false_verified_count = 0;//to be used in case we need to check whether a predicate is false

    if(!validCheckRequest())
        return false;
    
    for(ManagedBelief mb : mbArray)
    {
        bool check_res = performCheckAgainstBelief(mb);
        if(check_res)
        {
            if(check_ != c.FALSE_CHECK)
                return true;
            else
                false_verified_count++; // false needs to be tested against all current kb
        }

        if(check_ == c.FALSE_CHECK && !check_res)
            return false; // check false where found to be true
    }

    if (check_ == c.FALSE_CHECK && false_verified_count == mbArray.size())
            return true; // check false has been successfully verified against all kb

    return false;
}

bool ManagedCondition::performCheckAgainstBeliefs(const set<ManagedBelief>& mbSet)
{
    Condition c = Condition();
    uint8_t false_verified_count = 0;//to be used in case we need to check whether a predicate is false

    if(!validCheckRequest())
        return false;
    
    for(ManagedBelief mb : mbSet)
    {
        bool check_res = performCheckAgainstBelief(mb);
        if(check_res)
        {
            if(check_ != c.FALSE_CHECK)
                return true;
            else
                false_verified_count++; // false needs to be tested against all current kb
        }

        if(check_ == c.FALSE_CHECK && !check_res)
            return false; // check false where found to be true
    }

    if (check_ == c.FALSE_CHECK && false_verified_count == mbSet.size())
            return true; // check false has been successfully verified against all kb

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

std::ostream& BDIManaged::operator<<(std::ostream& os, const ManagedCondition& mc)
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

    os << "\nCHECK TO BE PERFORMED:" << check_string  << " \t CONDITION TO BE CHECKED: " << mc.getMGBelief();
    return os;
}

// overload `<` operator 
bool BDIManaged::operator<(const ManagedCondition& mc1, const ManagedCondition& mc2)
{
    return mc1.getCheck() < mc2.getCheck() && mc1.getMGBelief() < mc2.getMGBelief();
}

// overload `==` operator 
bool BDIManaged::operator==(const ManagedCondition& mc1, const ManagedCondition& mc2)
{
    return mc1.getCheck() == mc2.getCheck() && mc1.getMGBelief() == mc2.getMGBelief();
}   