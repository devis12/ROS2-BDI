#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/PDDLBDIConstants.hpp"

#include "plansys2_domain_expert/DomainReader.hpp"

#include <boost/algorithm/string.hpp>

#define INSTANCE_S PDDLBDIConstants::INSTANCE_TYPE
#define PREDICATE_S PDDLBDIConstants::PREDICATE_TYPE
#define FUNCTION_S PDDLBDIConstants::FUNCTION_TYPE

using std::string;
using std::vector;
using std::map;

using ros2_bdi_interfaces::msg::Belief;

using BDIManaged::ManagedBelief;

ManagedBelief::ManagedBelief():
    name_(""),
    pddl_type_(-1)
    {}

ManagedBelief::ManagedBelief(const std::string& name,const int& pddl_type, const std::string& type):
    name_(name),
    pddl_type_(pddl_type),
    type_(type)
{}
    
ManagedBelief::ManagedBelief(const std::string& name,const int& pddl_type,const std::vector<ManagedParam>& params, const float& value):
    name_(name),
    pddl_type_(pddl_type),
    params_(params),
    value_ (value)
    {
        if(pddl_type == Belief().PREDICATE_TYPE)
            type_ = PDDLBDIConstants::PREDICATE_TYPE;
        else if(pddl_type == Belief().FUNCTION_TYPE)
            type_ = PDDLBDIConstants::FUNCTION_TYPE;
    }

ManagedBelief::ManagedBelief(const Belief& belief):
    name_(belief.name),
    pddl_type_ (belief.pddl_type),
    type_(belief.type),
    value_ (belief.value)
    {
        for(string p : belief.params)
            params_.push_back(ManagedParam{p,""});
    }

// Clone a MG Belief DNF
ManagedBelief ManagedBelief::clone()
{
    string name = string{name_};
    if(pddl_type_ == Belief().INSTANCE_TYPE)
    {
        string instance_type = string{type_};
        return (ManagedBelief::buildMBInstance(name, instance_type));
    }
    else if(pddl_type_ == Belief().PREDICATE_TYPE || pddl_type_ == Belief().FUNCTION_TYPE)
    {
        vector<ManagedParam> params;
        for(auto p : params_)
            params.push_back(ManagedParam{string{p.name}, string{p.type}}); 
        
        float value = 0.0f;
        if(pddl_type_ == Belief().FUNCTION_TYPE)
            value = value_;

        return pddl_type_ == Belief().PREDICATE_TYPE?
             (ManagedBelief::buildMBPredicate(name, params))
             :
             (ManagedBelief::buildMBFunction(name, params, value));
    }
}

ManagedBelief ManagedBelief::buildMBInstance(const string& name, const string& instance_type)
{
    return ManagedBelief{name, Belief().INSTANCE_TYPE, instance_type};
}

ManagedBelief ManagedBelief::buildMBPredicate(const string& name, const vector<ManagedParam>& params)
{
    vector<ManagedParam> pred_params;
    for(ManagedParam p : params)
        pred_params.push_back(p);
    return ManagedBelief{name, Belief().PREDICATE_TYPE, pred_params, 0.0f};
}

ManagedBelief ManagedBelief::buildMBFunction(const string& name, const vector<ManagedParam>& params, const float& value)
{
    vector<ManagedParam> fun_params;
    for(ManagedParam p : params)
        fun_params.push_back(p);
    return ManagedBelief{name, Belief().FUNCTION_TYPE, fun_params, value};
}

Belief ManagedBelief::toBelief() const
{
    Belief b = Belief();
    b.name = name_;
    b.pddl_type = pddl_type_;
    b.type = type_;
    for(ManagedParam mp : params_)
        b.params.push_back(mp.name);
    b.value = value_;
    return b;
}

/*  convert instance to ros2_bdi_interfaces::msg::Belief format substituting name with respective fulfilling name, which
                indicates we're currently pursuing x (e.g. "f_x" for "x")*/
Belief ManagedBelief::toFulfillmentBelief() const
{
    Belief b = toBelief();
    b.name = FULFILLMENT_PREFIX + name_;

    return b;
}

/* substitute placeholders as per assignments map and return a new ManagedBelief instance*/
ManagedBelief ManagedBelief::applySubstitution(const map<string, string> assignments) const
{
    if(pddl_type_ == Belief().FUNCTION_TYPE || pddl_type_ == Belief().PREDICATE_TYPE)
    {
        vector<ManagedParam> params_subs;
        for(ManagedParam mp : params_)
            if(mp.isPlaceholder() && assignments.count(mp.name) == 1)
                params_subs.push_back(ManagedParam{assignments.find(mp.name)->second, mp.type});//replace param's name placeholder with assigned one
            else
                params_subs.push_back(ManagedParam{mp.name, mp.type});//no placeholder returns as is
                
        return ManagedBelief{name_, pddl_type_, params_subs, value_};
    }
    else if(pddl_type_ == Belief().INSTANCE_TYPE)
    {
        if(assignments.count(name_) == 1)
            return ManagedBelief{assignments.find(name_)->second, Belief().INSTANCE_TYPE, type_};//replace name placeholder with assigned one
        else
            return ManagedBelief{name_, Belief().INSTANCE_TYPE, type_};//no placeholder returns as is
    }

    return ManagedBelief{};
}

/*
    Get param list as a single joined string separated from spaces
*/
string ManagedBelief::getParamsJoined(const char separator) const
{
    string params_string = "";
    vector<ManagedParam> mb_params = this->getParams();
    for(int i=0; i<mb_params.size(); i++)
        params_string += (i==mb_params.size()-1)? mb_params[i].name : mb_params[i].name + separator;
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
            vector<string> params_string;
            vector<ManagedParam> params;
            boost::split(params_string, items[2], [](char c){return c == ' ';});//split string
            for(string par : params_string)
                params.push_back(ManagedParam{par, ""});//TODO fix this

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
    string param_or_type_string = "";
    if(mb.pddlType() == Belief().INSTANCE_TYPE)
        param_or_type_string = " " + mb.type();
    else
        for(ManagedParam p : mb.getParams())
            param_or_type_string += " " + p.name;

    os << mb.pddlType() << ":(" << mb.getName() << param_or_type_string + ")";
    
    if (mb.pddlType() == Belief().FUNCTION_TYPE)
        os << " value=" << mb.getValue();
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
    
    vector<ManagedParam> mb1_params = mb1.getParams();
    vector<ManagedParam> mb2_params = mb2.getParams();

    if(mb1_params.size() != mb2_params.size())
        return mb1_params.size() < mb2_params.size();

    //check equals param by param (at this point you know the two arrays are the same size)
    for(int i=0;i<mb1_params.size();i++)
        if(mb1_params[i].name != mb2_params[i].name)
            return mb1_params[i].name < mb2_params[i].name;


    return false;//do not check value_ (functions are considered the same if they just have diff. value_)
}

// overload `==` operator 
bool BDIManaged::operator==(ManagedBelief const &mb1, ManagedBelief const &mb2){
    //check for different types
    //do not check value_ (functions are considered the same if they just have diff. value_)
    if(mb1.pddlType() != mb2.pddlType() /* || (mb1.type_ == FUNCTION_TYPE && mb1.value_ != mb2.value_)*/)
        return false;

    if(mb1.pddlType() == Belief().INSTANCE_TYPE && mb1.type() != mb2.type())
        return false;

    vector<ManagedParam> mb1_params = mb1.getParams();
    vector<ManagedParam> mb2_params = mb2.getParams();

    //check for different name or different num of params
    if(mb1.getName() != mb2.getName() || mb1_params.size() != mb2_params.size()) // names OR sizes differ
        return false;


    //check equals param by param (at this point you know the two arrays are the same size)
    for(int i=0;i<mb1_params.size();i++)
        if(mb1_params[i].name != mb2_params[i].name) //params in pos i are different
            return false;

    //otherwise equals
    return true;
}