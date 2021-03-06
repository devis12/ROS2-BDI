#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/PDDLBDIConstants.hpp"

#include <boost/algorithm/string.hpp>

#define INSTANCE_S PDDLBDIConstants::INSTANCE_TYPE
#define PREDICATE_S PDDLBDIConstants::PREDICATE_TYPE
#define FUNCTION_S PDDLBDIConstants::FUNCTION_TYPE

using std::string;
using std::vector;

using ros2_bdi_interfaces::msg::Belief;

using BDIManaged::ManagedBelief;

ManagedBelief::ManagedBelief():
    name_(""),
    pddl_type_(-1)
    {
    }
    
ManagedBelief::ManagedBelief(const string& name,const int& type,const vector<string>& params,const float& value):
    name_(name),
    pddl_type_(type),
    params_(params),
    value_ (value)
    {
    }

ManagedBelief::ManagedBelief(const Belief& belief):
    name_(belief.name),
    pddl_type_ (belief.pddl_type),
    params_(belief.params),
    value_ (belief.value)
    {}

ManagedBelief ManagedBelief::buildMBInstance(const string& name, const string& instance_type)
{
    vector<string> ins_type_vec = vector<string>({instance_type});
    return ManagedBelief{name, Belief().INSTANCE_TYPE, ins_type_vec, 0.0f};
}

ManagedBelief ManagedBelief::buildMBPredicate(const string& name, const vector<string>& params)
{
    vector<string> pred_params;
    for(string p : params)
        pred_params.push_back(p);
    return ManagedBelief{name, Belief().PREDICATE_TYPE, pred_params, 0.0f};
}

ManagedBelief ManagedBelief::buildMBFunction(const string& name, const vector<string>& params, const float& value)
{
    vector<string> fun_params;
    for(string p : params)
        fun_params.push_back(p);
    return ManagedBelief{name, Belief().FUNCTION_TYPE, fun_params, value};
}

Belief ManagedBelief::toBelief() const
{
    Belief b = Belief();
    b.name = name_;
    b.pddl_type = pddl_type_;
    b.params = params_;
    b.value = value_;
    return b;
}

/*
    Get param list as a single joined string separated from spaces
*/
string ManagedBelief::getParamsJoined(const char separator) const
{
    string params_string = "";
    vector<string> mb_params = this->getParams();
    for(int i=0; i<mb_params.size(); i++)
        params_string += (i==mb_params.size()-1)? mb_params[i] : mb_params[i] + separator;
    return params_string;
}

string ManagedBelief::pddlTypeString() const
{
    string pddl_type_string =   (pddl_type_ == Belief().INSTANCE_TYPE)? INSTANCE_S  :
                            (pddl_type_ == Belief().PREDICATE_TYPE)? PREDICATE_S :
                            (pddl_type_ == Belief().FUNCTION_TYPE)? FUNCTION_S : "";
    return pddl_type_string;
}


/*
    Try to parse a managed belief from a string, format is the following
    assuming delimiters = ['(', ')']

    ({pddl_type}, {name}, {p1} {p2} {p3} ... {p54}, [{value}])
*/
std::optional<ManagedBelief> ManagedBelief::parseMGBelief(std::string mg_belief, const char delimiters[])
{
    int delimiterNum = sizeof(delimiters)/sizeof(delimiters[0]);
    if(delimiterNum != 2)
        return std::nullopt;
    else
    {
        vector<string> items;
        if(mg_belief.length() == 0)
            return std::nullopt;
        else
        {
            //remove parenthesis
            if(mg_belief.at(0) == delimiters[0])
                mg_belief = mg_belief.substr(1);
            if(mg_belief.at(mg_belief.length()-1) == delimiters[1])
                mg_belief = mg_belief.substr(0,mg_belief.length()-1);

            boost::split(items, mg_belief, [](char c){return c == ',';});//split string
            if(items.size() != 3 && items.size() != 4)//items should be either 3 or 4
                return std::nullopt;

            //retrieve elements
            int pddl_type = atoi(items[0].c_str());
            string name = items[1];
            vector<string> params;
            boost::split(params, items[2], [](char c){return c == ' ';});//split string
            
            float value = 0.0f;
            if(pddl_type == Belief().FUNCTION_TYPE && items.size() == 4)
                value = atof(items[3].c_str());
            
            return ManagedBelief{name, pddl_type, params, value};
        }
    }
}   

/*
    Convert managed belief to string, format is the following

    ({pddl_type},{name},{p1} {p2} {p3} ... {p54},[{value}])
*/
string ManagedBelief::toString(const char delimiters[]) const{
    //if delimiterNum is wrong, set delimiters to default values ['(',')']
    int delimiterNum = sizeof(delimiters)/sizeof(delimiters[0]);
    char d0 = '(', d1 = ')';
    if(delimiterNum != 2)
    {
        d0 = '(';
        d1 = ')';
    }

    string result = d0 + std::to_string(pddl_type_) + "," + name_ + "," + getParamsJoined();
    if(pddl_type_ == Belief().FUNCTION_TYPE)
        result += "," + std::to_string(value_);
    
    result += d1;

    return result;
}

std::ostream& BDIManaged::operator<<(std::ostream& os, const ManagedBelief& mb)
{
    string param_string = "";
    for(string p : mb.getParams())
        param_string += p + " ";

    os << mb.pddlType() << ":( " << mb.getName() <<  " " + param_string + ") " << mb.getValue();
    return os;
}

// overload `<` operator 
bool BDIManaged::operator<(ManagedBelief const &mb1, ManagedBelief const &mb2)
{
    if(mb1.pddlType() != mb2.pddlType())
    {
        //for arbitrary order: INSTANCE comes first, then PREDICATE and after that FUNCTION_TYPE
        if(mb1.pddlType() == Belief().INSTANCE_TYPE || mb1.pddlType() == Belief().PREDICATE_TYPE && mb2.pddlType() == Belief().FUNCTION_TYPE)
            return true;
        else
            return false;   
    }

    if(mb1.getName() != mb2.getName())
        return mb1.getName() < mb2.getName();
    
    vector<string> mb1_params = mb1.getParams();
    vector<string> mb2_params = mb2.getParams();

    if(mb1_params.size() != mb2_params.size())
        return mb1_params.size() < mb2_params.size();

    //check equals param by param (at this point you know the two arrays are the same size)
    for(int i=0;i<mb1_params.size();i++)
        if(mb1_params[i] != mb2_params[i])
            return mb1_params[i] < mb2_params[i];


    return false;//do not check value_ (functions are considered the same if they just have diff. value_)
}

// overload `==` operator 
bool BDIManaged::operator==(ManagedBelief const &mb1, ManagedBelief const &mb2){
    //check for different types
    //do not check value_ (functions are considered the same if they just have diff. value_)
    if(mb1.pddlType() != mb2.pddlType() /* || (mb1.type_ == FUNCTION_TYPE && mb1.value_ != mb2.value_)*/)
        return false;

    vector<string> mb1_params = mb1.getParams();
    vector<string> mb2_params = mb2.getParams();

    //check for different name or different num of params
    if(mb1.getName() != mb2.getName() || mb1_params.size() != mb2_params.size()) // names OR sizes differ
        return false;

    //check equals param by param (at this point you know the two arrays are the same size)
    for(int i=0;i<mb1_params.size();i++)
        if(mb1_params[i] != mb2_params[i]) //params in pos i are different
            return false;

    //otherwise equals
    return true;
}