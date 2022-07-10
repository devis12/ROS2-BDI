#include "ros2_bdi_utils/ManagedConditionsDNF.hpp"

#include <boost/algorithm/string.hpp>

using std::vector;
using std::set;
using std::string;

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

/*
    Try to parse a ManagedConditionsDNF from a string, format is the following for ConditionsDNF

    Condition(literal) = ({check}/{{pddl_type},{name},{p1} {p2} {p3} ... {p54},[{value}]})
    ConditionsConjunction(clause) = (l1&l2&l3)
    ConditionsDNF(expression) = clause1|clause2
*/
std::optional<ManagedConditionsDNF> ManagedConditionsDNF::parseMGConditionsDNF(string mg_conditions_dnf)
{
    if(mg_conditions_dnf.length() == 0)
        return std::nullopt;
    else
    {   
        //clauses (string and actual objs)
        vector<string> s_clauses;
        vector<ManagedConditionsConjunction> clauses;

        boost::split(s_clauses, mg_conditions_dnf, [](char c){return c == '|';});//split string
        for(string s_clause : s_clauses)
        {
            //literals (string and actual objs)
            vector<string> s_literals;
            vector<ManagedCondition> literals;
            boost::split(s_literals, s_clause, [](char c){return c == '&';});//split string
            for(string s_literal : s_literals)
            {
                vector<string> condition_items;
                boost::split(condition_items, s_literal, [](char c){return c == '/';});//split string
                if(condition_items.size() != 2)
                    return std::nullopt;
                else
                {
                    string check = condition_items[0];
                    std::optional<ManagedBelief> mg_belief = ManagedBelief::parseMGBelief(condition_items[1], mgcond_belief_default_delimiters);
                    if(!mg_belief.has_value())
                        return std::nullopt;
                    else
                        literals.push_back(ManagedCondition{mg_belief.value(), check});
                }
            }
            clauses.push_back(ManagedConditionsConjunction{literals});
        }

        return ManagedConditionsDNF(clauses);
    }
}

/*
    Convert ManagedConditionsDNF to string, format is the following

    Condition(literal) = ({check}/{{pddl_type},{name},{p1} {p2} {p3} ... {p54},[{value}]})
    ConditionsConjunction(clause) = (l1&l2&l3)
    ConditionsDNF(expression) = clause1|clause2
*/
string ManagedConditionsDNF::toString() const
{
    int clause_num=0;
    int literal_num=0;
    string result = "";
    for(ManagedConditionsConjunction clause : clauses_)
    {
        result += mgcond_clause_default_delimiters[0];
        for(ManagedCondition literal : clause.getLiterals())
        {
            string belief_to_check = literal.getMGBelief().toString(mgcond_belief_default_delimiters);
            result += belief_to_check;
            if(literal_num < clause.getLiterals().size()-1)//avoid to put & after last literal in clause
                result += "&";
            literal_num++;
        }
        result += mgcond_clause_default_delimiters[1];
        if(clause_num < clauses_.size()-1)//avoid to put & after last clause in DNF expression
                result += "|";
        clause_num++;
    }
    return result;
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
