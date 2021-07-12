#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/PDDLBDIConstants.hpp"

#define FLUENT_TYPE PDDLBDIConstants::FLUENT_TYPE
#define PREDICATE_TYPE PDDLBDIConstants::PREDICATE_TYPE


ManagedBelief::ManagedBelief(const string& name,const string& type,const vector<string>& params,const float& value):
    name_(name),
    type_ (type),
    params_(params),
    value_ (value)
    {}

ManagedBelief::ManagedBelief(const Belief& belief):
    name_(belief.name),
    type_ (belief.type),
    params_(belief.params),
    value_ (belief.value)
    {}

Belief ManagedBelief::toBelief() const
{
    Belief b = Belief();
    b.name = name_;
    b.type = type_;
    b.params = params_;
    b.value = value_;
    return b;
}

std::ostream& operator<<(std::ostream& os, const ManagedBelief& mb)
{
    string param_string = "";
    for(string p : mb.params_)
        param_string += p + " ";

    os << mb.type_  << ":( " << mb.name_ <<  " " + param_string + ") " << mb.value_;
    return os;
}

// overload `<` operator 
bool operator<(ManagedBelief const &mb1, ManagedBelief const &mb2)
{
    if(mb1.type_ != mb2.type_)
        return mb1.type_ == PREDICATE_TYPE;

    if(mb1.name_ != mb2.name_)
        return mb1.name_ < mb2.name_;
    
    if(mb1.params_.size() != mb2.params_.size())
        return mb1.params_.size() < mb2.params_.size();

    //check equals param by param (at this point you know the two arrays are the same size)
    for(int i=0;i<mb1.params_.size();i++)
        if(mb1.params_[i] != mb2.params_[i])
            return mb1.params_[i] < mb2.params_[i];


    return false;//do not check value_ (fluents are considered the same if they just have diff. value_)
}

// overload `==` operator 
bool operator==(ManagedBelief const &mb1, ManagedBelief const &mb2){
    //check for different types
    //do not check value_ (fluents are considered the same if they just have diff. value_)
    if(mb1.type_ != mb2.type_ /* || (mb1.type_ == FLUENT_TYPE && mb1.value_ != mb2.value_)*/)
        return false;

    //check for different name or different num of params
    if(mb1.name_ != mb2.name_ || mb1.params_.size() != mb2.params_.size())
        return false;

    //check equals param by param (at this point you know the two arrays are the same size)
    for(int i=0;i<mb1.params_.size();i++)
        if(mb1.params_[i] != mb2.params_[i])
            return false;

    //otherwise equals
    return true;
}