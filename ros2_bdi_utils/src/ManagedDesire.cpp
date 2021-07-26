#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"


ManagedDesire::ManagedDesire():
    name_(""),
    priority_(0.0f),
    value_(vector<ManagedBelief>()),
    deadline_(0.0f)
    {}


ManagedDesire::ManagedDesire(const string& name,const vector<ManagedBelief>& value,const float& priority,const float& deadline):
    name_(name),
    value_ (value),
    priority_(priority),
    deadline_(deadline)
    {}

ManagedDesire::ManagedDesire(const Desire& desire):
    name_(desire.name),
    priority_(desire.priority),
    deadline_(desire.deadline)
    {   
        set<ManagedBelief> set_mb = BDIFilter::extractMGPredicates(desire.value);
        value_ = vector<ManagedBelief>(set_mb.begin(), set_mb.end());
    }

Desire ManagedDesire::toDesire() const
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
    os << md.getName();

    for(ManagedBelief mb : md.getValue())
        os << mb << " ";

    os << "\n P:" << md.getPriority() << "\tD:" << md.getDeadline();
    return os;
}

// overload `<` operator 
bool operator<(ManagedDesire const &md1, ManagedDesire const &md2)
{   
    // first check based on "simple" values (name, priority, deadline)
    if(md1.getName() < md2.getName() || md1.getPriority() < md2.getPriority() || md1.getDeadline() < md2.getDeadline())
        return true;
    
    vector<ManagedBelief> md1_value = md1.getValue();
    vector<ManagedBelief> md2_value = md2.getValue();

    // check based # of mg. beliefs in value 
    if(md1_value.size() < md2_value.size())
        return true;

    // order do not count, so put them into two sets
    set<ManagedBelief> md1_beliefs;
    for(ManagedBelief mb : md1_value)
        md1_beliefs.insert(mb);
    set<ManagedBelief> md2_beliefs;
    for(ManagedBelief mb : md2_value)
        md2_beliefs.insert(mb);

    // check for every belief of one mg. desire if it is contained also in the other mg. desire
    for(ManagedBelief mb1 : md1_beliefs)
        if(md2_beliefs.count(mb1)==0)
            return true;

    return false; //otherwise return false
}

// overload `==` operator 
bool operator==(ManagedDesire const &md1, ManagedDesire const &md2){
     // first check based on "simple" values (name, priority, deadline)
    if(md1.getName() != md2.getName() || md1.getPriority() != md2.getPriority() || md1.getDeadline() != md2.getDeadline())
        return false;
    
    vector<ManagedBelief> md1_value = md1.getValue();
    vector<ManagedBelief> md2_value = md2.getValue();

    // check based # of mg. beliefs in value 
    if(md1_value.size() < md2_value.size())
        return false;

    // order do not count, so put them into two sets
    set<ManagedBelief> md1_beliefs;
    for(ManagedBelief mb : md1_value)
        md1_beliefs.insert(mb);
    set<ManagedBelief> md2_beliefs;
    for(ManagedBelief mb : md2_value)
        md2_beliefs.insert(mb);

    // check for every belief of one mg. desire if it is contained also in the other mg. desire
    for(ManagedBelief mb1 : md1_beliefs)
        if(md2_beliefs.count(mb1)==0)
            return false;

    //otherwise equals
    return true;
}