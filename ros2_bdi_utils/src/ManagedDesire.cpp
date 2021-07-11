#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"


ManagedDesire::ManagedDesire(string name, vector<ManagedBelief> value, float priority, float deadline):
    name_(name),
    value_ (value),
    priority_(priority),
    deadline_(deadline)
    {}

ManagedDesire::ManagedDesire(Desire desire):
    name_(desire.name),
    priority_(desire.priority),
    deadline_(desire.deadline)
    {   
        set<ManagedBelief> set_mb = BDIFilter::extractMGPredicates(desire.value);
        value_ = vector<ManagedBelief>(set_mb.begin(), set_mb.end());
    }

Desire ManagedDesire::toDesire()
{
    Desire d = Desire();
    d.name = name_;
    vector<Belief> target_beliefs = vector<Belief>();
    for(ManagedBelief mb : value_)
        target_beliefs.push_back(mb.toBelief());
    d.value = target_beliefs;
    d.priority = priority_;
    d.deadline = deadline_;
    return d;
}

std::ostream& operator<<(std::ostream& os, const ManagedDesire& md)
{
    os << md.name_;

    for(ManagedBelief mb : md.value_)
        os << mb << " ";

    os << "\n P:" << md.priority_ << "\tD:" << md.deadline_;
    return os;
}

// overload `<` operator 
bool operator<(ManagedDesire const &md1, ManagedDesire const &md2)
{   
    // first check based on "simple" values (name, priority, deadline)
    if(md1.name_ < md2.name_ || md1.priority_ < md2. priority_ || md1.deadline_ < md2.deadline_)
        return true;
    
    // check based # of mg. beliefs in value 
    if(md1.value_.size() < md2.value_.size())
        return true;

    // check in order mg. belief by  mg. belief
    for(int i=0; i<md1.value_.size(); i++)
        if(md1.value_[i] < md2.value_[i])
            return true;

    return false; //otherwise return false
}

// overload `==` operator 
bool operator==(ManagedDesire const &md1, ManagedDesire const &md2){
     // first check based on "simple" values (name, priority, deadline)
    if(md1.name_ != md2.name_ || md1.priority_ != md2. priority_ || md1.deadline_ != md2.deadline_)
        return false;
    
    // check based # of mg. beliefs in value 
    if(md1.value_.size() < md2.value_.size())
        return false;

    // check in order mg. belief by  mg. belief
    for(int i=0; i<md1.value_.size(); i++)
        if(!(md1.value_[i] == md2.value_[i]))
            return false;

    //otherwise equals
    return true;
}