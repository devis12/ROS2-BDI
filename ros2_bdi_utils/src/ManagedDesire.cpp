#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"


ManagedDesire::ManagedDesire():
    name_(""),
    desire_group_(""),
    priority_(0.0f),
    value_(vector<ManagedBelief>()),
    deadline_(0.0f),
    precondition_(ManagedConditionsDNF()),
    context_(ManagedConditionsDNF())
    {}


ManagedDesire::ManagedDesire(const string& name,const vector<ManagedBelief>& value,const float& priority,const float& deadline):
    name_(name),
    desire_group_(name),
    value_ (value),
    priority_(priority),
    deadline_(deadline),
    precondition_(ManagedConditionsDNF()),
    context_(ManagedConditionsDNF())
    {
        if(priority_ < 0.0f)
            priority_ = 0.0f;
        
        if(deadline_ < 0.0f)
            deadline_ = 0.0f;
    }


ManagedDesire::ManagedDesire(const string& name, const string& desire_group, const vector<ManagedBelief>& value,const float& priority,const float& deadline):
    name_(name),
    desire_group_(desire_group),
    value_ (value),
    priority_(priority),
    deadline_(deadline),
    precondition_(ManagedConditionsDNF()),
    context_(ManagedConditionsDNF())
    {
        if(priority_ < 0.0f)
            priority_ = 0.0f;
        
        if(deadline_ < 0.0f)
            deadline_ = 0.0f;
    }

ManagedDesire::ManagedDesire(const string& name,const vector<ManagedBelief>& value,const float& priority,const float& deadline,
                        const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context):
    name_(name),
    desire_group_(name),
    value_ (value),
    priority_(priority),
    deadline_(deadline),
    precondition_(precondition),
    context_(context)
    {
        if(priority_ < 0.0f)
            priority_ = 0.0f;
        
        if(deadline_ < 0.0f)
            deadline_ = 0.0f;
    }


ManagedDesire::ManagedDesire(const Desire& desire):
    name_(desire.name),
    desire_group_(desire.name),
    priority_(desire.priority),
    deadline_(desire.deadline)
    {   
        if(priority_ < 0.0f)
            priority_ = 0.0f;
        
        if(deadline_ < 0.0f)
            deadline_ = 0.0f;

        set<ManagedBelief> set_mb = BDIFilter::extractMGPredicates(desire.value);
        value_ = vector<ManagedBelief>(set_mb.begin(), set_mb.end());
        
        precondition_ = ManagedConditionsDNF{desire.precondition};
        context_ = ManagedConditionsDNF{desire.context};
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

    d.precondition = precondition_.toConditionsDNF();

    d.context = context_.toConditionsDNF();

    return d;
}

std::ostream& operator<<(std::ostream& os, const ManagedDesire& md)
{
    os << "\n" << md.getName();

    for(ManagedBelief mb : md.getValue())
        os << "\n" << mb ;

    os << "\n Priority: " << md.getPriority() << "\tDeadline:" << md.getDeadline();
    
    os << "\n Preconditions: " << md.getPrecondition();

    os << "\n Context: " << md.getContext();
    
    return os;
}

// overload `<` operator 
bool operator<(ManagedDesire const &md1, ManagedDesire const &md2)
{   
    // first check based on "simple" values (name, priority, deadline)
    if(md1.getName() != md2.getName())
        return md1.getName() < md2.getName();

    else if(md1.getPriority() != md2.getPriority() && (md1.getPriority()-md2.getPriority()) > 0.01f)//lower digit diff do not count as actual difference
        return md1.getPriority() < md2.getPriority();
    
    else if(md1.getDeadline() != md2.getDeadline() && (md1.getDeadline()-md2.getDeadline()) > 0.01f)//lower digit diff do not count as actual difference
        return md1.getDeadline() < md2.getDeadline();
    
    vector<ManagedBelief> md1_value = md1.getValue();
    vector<ManagedBelief> md2_value = md2.getValue();

    // check based # of mg. beliefs in value 
    if(md1_value.size() < md2_value.size())
        return true;

    // order do not count, so put them into two sets
    set<ManagedBelief> md1_beliefS;
    for(ManagedBelief mb : md1_value)
        md1_beliefS.insert(mb);
    set<ManagedBelief> md2_beliefS;
    for(ManagedBelief mb : md2_value)
        md2_beliefS.insert(mb);

    // check for every belief of one mg. desire if it is contained also in the other mg. desire
    for(ManagedBelief mb1 : md1_beliefS)
        if(md2_beliefS.count(mb1)==0)
            return true;
    
    if(md1.getPrecondition() < md2.getPrecondition() || md1.getContext() < md1.getContext())
        return true;

    return false; //otherwise return false
}

// overload `==` operator 
bool operator==(ManagedDesire const &md1, ManagedDesire const &md2){
     // first check based on "simple" values (name, priority, deadline)
    if(md1.getName() != md2.getName())
        return false;

    else if(md1.getPriority() != md2.getPriority() && (md1.getPriority()-md2.getPriority()) > 0.01f)//lower digit diff do not count as actual difference
        return false;
    
    else if(md1.getDeadline() != md2.getDeadline() && (md1.getDeadline()-md2.getDeadline()) > 0.01f)//lower digit diff do not count as actual difference
        return false;
    
    vector<ManagedBelief> md1_value = md1.getValue();
    vector<ManagedBelief> md2_value = md2.getValue();

    // check based # of mg. beliefs in value 
    if(md1_value.size() < md2_value.size())
        return false;

    // order do not count, so put them into two sets
    set<ManagedBelief> md1_beliefS;
    for(ManagedBelief mb : md1_value)
        md1_beliefS.insert(mb);
    set<ManagedBelief> md2_beliefS;
    for(ManagedBelief mb : md2_value)
        md2_beliefS.insert(mb);

    // check for every belief of one mg. desire if it is contained also in the other mg. desire
    for(ManagedBelief mb1 : md1_beliefS)
        if(md2_beliefS.count(mb1)==0)
            return false;

    //otherwise compare precondition & context
    return md1.getPrecondition() == md2.getPrecondition() && md1.getContext() == md1.getContext();
}