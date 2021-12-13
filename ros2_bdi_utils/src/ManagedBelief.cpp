#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/PDDLBDIConstants.hpp"

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

string ManagedBelief::pddlTypeString() const
{
    string pddl_type_string =   (pddl_type_ == Belief().INSTANCE_TYPE)? INSTANCE_S  :
                            (pddl_type_ == Belief().PREDICATE_TYPE)? PREDICATE_S :
                            (pddl_type_ == Belief().FUNCTION_TYPE)? FUNCTION_S : "";
    return pddl_type_string;
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
    if(mb1.getName() != mb2.getName() || mb1_params.size() != mb2_params.size())
        return false;

    //check equals param by param (at this point you know the two arrays are the same size)
    for(int i=0;i<mb1_params.size();i++)
        if(mb1_params[i] != mb2_params[i])
            return false;

    //otherwise equals
    return true;
}