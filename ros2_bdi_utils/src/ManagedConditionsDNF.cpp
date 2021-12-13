#include "ros2_bdi_utils/ManagedConditionsDNF.hpp"

using std::vector;
using std::set;

using ros2_bdi_interfaces::msg::ConditionsConjunction;
using ros2_bdi_interfaces::msg::ConditionsDNF;

using BDIManaged::ManagedCondition;
using BDIManaged::ManagedConditionsConjunction;
using BDIManaged::ManagedConditionsDNF;

ManagedConditionsDNF::ManagedConditionsDNF():
    clauses_(vector<ManagedConditionsConjunction>()){}

ManagedConditionsDNF::ManagedConditionsDNF(const ConditionsDNF& conditionsDNF)
{
    clauses_ = vector<ManagedConditionsConjunction>();
    for(ConditionsConjunction cc : conditionsDNF.clauses)
        clauses_.push_back(ManagedConditionsConjunction{cc});  
}


ConditionsDNF ManagedConditionsDNF::toConditionsDNF() const
{
    ConditionsDNF c_dnf = ConditionsDNF();
    vector<ConditionsConjunction> clauses;
    for(ManagedConditionsConjunction mcc : clauses_)
        clauses.push_back(mcc.toConditionsConjunction());
    c_dnf.clauses = clauses;
    return c_dnf;
}

ManagedConditionsDNF::ManagedConditionsDNF(const vector<ManagedConditionsConjunction>& clauses):
    clauses_(clauses){}

bool ManagedConditionsDNF::isSatisfied(const set<ManagedBelief>& mbSet){
    for(ManagedConditionsConjunction mcc : clauses_)
        if(mcc.isSatisfied(mbSet))
            return true;
    
    return clauses_.size() == 0;// empty clause or no single clause is satisfied
}

std::ostream& BDIManaged::operator<<(std::ostream& os, const ManagedConditionsDNF& mcdnf)
{
    auto clauses = mcdnf.getClauses();
    
    os << "(OR\n";
    if(clauses.size() == 0 )
        os << " (TRUE) ";
    
    for(auto mcc : clauses)
        os << " (" << mcc << ") ";

    os << "\n)";
    return os;
}

// overload `<` operator 
bool BDIManaged::operator<(const ManagedConditionsDNF& mcdnf1, const ManagedConditionsDNF& mcdnf2)
{
    vector<ManagedConditionsConjunction> mcdnf1_clauses = mcdnf1.getClauses();
    vector<ManagedConditionsConjunction> mcdnf2_clauses = mcdnf2.getClauses();

    if(mcdnf1_clauses.size() != mcdnf2_clauses.size())
        return mcdnf1_clauses.size() < mcdnf2_clauses.size();
    
    // when sizes are equivalent order is not important
    // use sort to order them singularly
    std::sort(mcdnf1_clauses.begin(), mcdnf1_clauses.end());
    std::sort(mcdnf2_clauses.begin(), mcdnf2_clauses.end());

    for(int i=0; i<mcdnf1_clauses.size(); i++)
        if(mcdnf1_clauses[i] < mcdnf2_clauses[i])
            return true;
    
    return false;
}

// overload `==` operator 
bool BDIManaged::operator==(const ManagedConditionsDNF& mcdnf1, const ManagedConditionsDNF& mcdnf2)
{
    vector<ManagedConditionsConjunction> mcdnf1_clauses = mcdnf1.getClauses();
    vector<ManagedConditionsConjunction> mcdnf2_clauses = mcdnf1.getClauses();

    if(mcdnf1_clauses.size() != mcdnf2_clauses.size())
        return false;

    // build two sets because when sizes are equivalent order is not important
    set<ManagedConditionsConjunction> mcdnf1_clauses_set; 
    for(ManagedConditionsConjunction mcc1 : mcdnf1_clauses)
        mcdnf1_clauses_set.insert(mcc1);
    set<ManagedConditionsConjunction> mcdnf2_clauses_set; 
    for(ManagedConditionsConjunction mcc2 : mcdnf2_clauses)
        mcdnf2_clauses_set.insert(mcc2);

    for(ManagedConditionsConjunction mcc : mcdnf1_clauses_set)
        if(mcdnf2_clauses_set.count(mcc) == 0)//not found in mcc2 literal set, thus they are different
            return false;
    
    return true;
}
