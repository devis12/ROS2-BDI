#include "ros2_bdi_utils/ManagedConditionsConjunction.hpp"

#include <algorithm>

using std::vector;
using std::set;

using ros2_bdi_interfaces::msg::Condition;
using ros2_bdi_interfaces::msg::ConditionsConjunction;

using BDIManaged::ManagedCondition;
using BDIManaged::ManagedConditionsConjunction;

ManagedConditionsConjunction::ManagedConditionsConjunction():
    literals_(vector<ManagedCondition>()){}

ManagedConditionsConjunction::ManagedConditionsConjunction(const ConditionsConjunction& conditionsConjunction)
{
    literals_ = vector<ManagedCondition>();
    for(Condition c : conditionsConjunction.literals)
        literals_.push_back(ManagedCondition{c});  
}

ConditionsConjunction ManagedConditionsConjunction::toConditionsConjunction() const
{
    ConditionsConjunction cc = ConditionsConjunction();
    vector<Condition> literals;
    for(ManagedCondition mc : literals_)
        literals.push_back(mc.toCondition());
    cc.literals = literals;
    return cc;
}

ManagedConditionsConjunction::ManagedConditionsConjunction(const vector<ManagedCondition>& literals):
    literals_(literals){}

bool ManagedConditionsConjunction::isSatisfied(const set<ManagedBelief>& mbSet){
    return ManagedCondition::verifyAllManagedConditions(literals_, mbSet);//note: returns true if empty
}

std::ostream& BDIManaged::operator<<(std::ostream& os, const ManagedConditionsConjunction& mcc)
{
    auto literals = mcc.getLiterals();
    
    os << "(AND\n";
    if(literals.size() == 0 )
        os << " (TRUE) ";
    
    for(auto mc : literals)
        os << " (" << mc << ") ";

    os << "\n)";
    return os;
}

// overload `<` operator 
bool BDIManaged::operator<(const ManagedConditionsConjunction& mcc1, const ManagedConditionsConjunction& mcc2)
{
    vector<ManagedCondition> mcc1_literals = mcc1.getLiterals();
    vector<ManagedCondition> mcc2_literals = mcc2.getLiterals();

    if(mcc1_literals.size() != mcc2_literals.size())
        return mcc1_literals.size() < mcc2_literals.size();
    
    // when sizes are equivalent order is not important
    // use sort to order them singularly
    std::sort(mcc1_literals.begin(), mcc1_literals.end());
    std::sort(mcc2_literals.begin(), mcc2_literals.end());

    for(int i=0; i<mcc1_literals.size(); i++)
        if(mcc1_literals[i] < mcc2_literals[i])
            return true;
    
    return false;
}

// overload `==` operator 
bool BDIManaged::operator==(const ManagedConditionsConjunction& mcc1, const ManagedConditionsConjunction& mcc2){
    vector<ManagedCondition> mcc1_literals = mcc1.getLiterals();
    vector<ManagedCondition> mcc2_literals = mcc2.getLiterals();

    if(mcc1_literals.size() != mcc2_literals.size())
        return false;
    
    // build two sets because when sizes are equivalent order is not important
    set<ManagedCondition> mcc1_literals_set; 
    for(ManagedCondition mc1 : mcc1_literals)
        mcc1_literals_set.insert(mc1);
    set<ManagedCondition> mcc2_literals_set; 
    for(ManagedCondition mc2 : mcc2_literals)
        mcc2_literals_set.insert(mc2);

    for(ManagedCondition mc : mcc1_literals_set)
        if(mcc2_literals_set.count(mc) == 0)//not found in mcc2 literal set, thus they are different
            return false;
    
    return true;
}