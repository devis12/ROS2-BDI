#include "ros2_bdi_utils/PDDLUtils.hpp"

#include <boost/algorithm/string.hpp>

using std::vector;
using std::string;

/*Remove ALL parenthesis from an expression*/
string removeParenthesis(const string& expression)
{
    string stringNoPar = string(expression);
    while(stringNoPar.find("(") != string::npos)
        stringNoPar = stringNoPar.substr(stringNoPar.find("(")+1);
    while(stringNoPar.find(")") != string::npos)
        stringNoPar = stringNoPar.substr(0,stringNoPar.find(")"));
    return stringNoPar;
}

namespace PDDLUtils{

    /*
        Returns all the items within a PlanItem.action string
        E.g. "(dosweep sweeper kitchen)" -> ["dosweep", "sweeper", "kitchen"]
    */
    vector<string> extractPlanItemActionElements(const string& planItemAction)
    {
        vector<string> elems;
        if(planItemAction.length() > 0)
        {
            string planItemActionNoPar = removeParenthesis(planItemAction);

            boost::split(elems, planItemActionNoPar, [](char c){return c == ' ';});//split string
        }
        return elems;
    }

};