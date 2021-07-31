#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"


ManagedDesire::ManagedDesire():
    name_(""),
    priority_(0.0f),
    value_(vector<ManagedBelief>()),
    deadline_(0.0f),
    precondition_(vector<ManagedCondition>()),
    context_(vector<ManagedCondition>())
    {}


ManagedDesire::ManagedDesire(const string& name,const vector<ManagedBelief>& value,const float& priority,const float& deadline):
    name_(name),
    value_ (value),
    priority_(priority),
    deadline_(deadline),
    precondition_(vector<ManagedCondition>()),
    context_(vector<ManagedCondition>())
    {}

ManagedDesire::ManagedDesire(const string& name,const vector<ManagedBelief>& value,const float& priority,const float& deadline,
                        const vector<ManagedCondition>& precondition, const vector<ManagedCondition>& context):
    name_(name),
    value_ (value),
    priority_(priority),
    deadline_(deadline),
    precondition_(precondition),
    context_(context)
    {}

ManagedDesire::ManagedDesire(const Desire& desire):
    name_(desire.name),
    priority_(desire.priority),
    deadline_(desire.deadline)
    {   
        set<ManagedBelief> set_mb = BDIFilter::extractMGPredicates(desire.value);
        value_ = vector<ManagedBelief>(set_mb.begin(), set_mb.end());
        
        precondition_ = vector<ManagedCondition>();
        for(Condition c : desire.precondition)
            precondition_.push_back(ManagedCondition{c});
        
        context_ = vector<ManagedCondition>();
        for(Condition c : desire.context)
            context_.push_back(ManagedCondition{c});
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

    vector<Condition> precondition = vector<Condition>();
    for(ManagedCondition mc : precondition_)
            precondition.push_back(mc.toCondition());
    d.precondition = precondition;

    vector<Condition> context = vector<Condition>();
    for(ManagedCondition mc : context_)
            context.push_back(mc.toCondition());
    d.context = context;

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
    
    vector<ManagedCondition> md1_precondition = md1.getPrecondition();
    vector<ManagedCondition> md2_precondition = md2.getPrecondition();

    vector<ManagedCondition> md1_context = md1.getContext();
    vector<ManagedCondition> md2_context = md2.getContext();

    // check based # of mg. condition(s) in preconditions and context conditions 
    if(md1_precondition.size() < md2_precondition.size() || md1_context.size() < md2_context.size())
        return true;

    // order do not count, so put them into four sets
    
    set<ManagedCondition> md1_preconditionS;
    for(ManagedCondition mc : md1_precondition)
        md1_preconditionS.insert(mc);
    set<ManagedCondition> md1_contextS;
    for(ManagedCondition mc : md1_context)
        md1_contextS.insert(mc);

    set<ManagedCondition> md2_preconditionS;
    for(ManagedCondition mc : md2_precondition)
        md2_preconditionS.insert(mc);
    set<ManagedCondition> md2_contextS;
    for(ManagedCondition mc : md2_context)
        md2_contextS.insert(mc);

    // check for every mg condition of one mg. desire if it is contained also in the other mg. desire
    for(ManagedCondition mc1 : md1_preconditionS)
        if(md2_preconditionS.count(mc1)==0)
            return false;

    // check for every mg condition of one mg. desire if it is contained also in the other mg. desire
    for(ManagedCondition mc1 : md1_contextS)
        if(md2_contextS.count(mc1)==0)
            return false;

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

    vector<ManagedCondition> md1_precondition = md1.getPrecondition();
    vector<ManagedCondition> md2_precondition = md2.getPrecondition();

    vector<ManagedCondition> md1_context = md1.getContext();
    vector<ManagedCondition> md2_context = md2.getContext();

    // check based # of mg. condition(s) in preconditions and context conditions 
    if(md1_precondition.size() < md2_precondition.size() || md1_context.size() < md2_context.size())
        return true;

    // order do not count, so put them into four sets
    set<ManagedCondition> md1_preconditionS;
    for(ManagedCondition mc : md1_precondition)
        md1_preconditionS.insert(mc);
    set<ManagedCondition> md1_contextS;
    for(ManagedCondition mc : md1_context)
        md1_contextS.insert(mc);

    set<ManagedCondition> md2_preconditionS;
    for(ManagedCondition mc : md2_precondition)
        md2_preconditionS.insert(mc);
    set<ManagedCondition> md2_contextS;
    for(ManagedCondition mc : md2_context)
        md2_contextS.insert(mc);

    // check for every mg condition of one mg. desire if it is contained also in the other mg. desire
    for(ManagedCondition mc1 : md1_preconditionS)
        if(md2_preconditionS.count(mc1)==0)
            return false;

    // check for every mg condition of one mg. desire if it is contained also in the other mg. desire
    for(ManagedCondition mc1 : md1_contextS)
        if(md2_contextS.count(mc1)==0)
            return false;

    //otherwise equals
    return true;
}