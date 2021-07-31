#include "ros2_bdi_utils/ManagedPlan.hpp"

//#include "plansys2_domain_expert/DomainExpertClient.hpp"
//#include "plansys2_msgs/msg/Action"

//using plansys2::DomainExpertClient;
//using plansys2_msgs::msg::Action;

#define NO_PLAN "NO_PLAN"

ManagedPlan::ManagedPlan():
    body_(vector<PlanItem>()),
    precondition_(vector<ManagedCondition>()),
    context_(vector<ManagedCondition>()),
    plan_deadline_(0.0f)
    {
        Desire d = Desire{};
        d.name = NO_PLAN;
        desire_ = d;
    }


ManagedPlan::ManagedPlan(const ManagedDesire& md, const vector<PlanItem>& planitems):
    desire_(md),
    body_(planitems),
    precondition_(vector<ManagedCondition>()),
    context_(vector<ManagedCondition>())
    {   
        plan_deadline_ = computeDeadline(planitems);
    }

ManagedPlan::ManagedPlan(const ManagedDesire& md, const vector<PlanItem>& planitems, 
    const vector<ManagedCondition>& precondition, const vector<ManagedCondition>& context):
    desire_(md),
    body_(planitems),
    precondition_(precondition),
    context_(context)
    {   
        plan_deadline_ = computeDeadline(planitems);
    }

Plan ManagedPlan::toPsys2Plan() const
{
    Plan p = Plan();
    p.items =  body_;
    return p;
}

BDIPlan ManagedPlan::toPlan() const
{
    BDIPlan p = BDIPlan();
    p.desire = desire_.toDesire();
    p.actions = body_;

    vector<Condition> preconditions_msg = vector<Condition>();
    for(ManagedCondition mc : precondition_)
        preconditions_msg.push_back(mc.toCondition());
    p.precondition = preconditions_msg;

    vector<Condition> context_msg = vector<Condition>();
    for(ManagedCondition mc : context_)
        context_msg.push_back(mc.toCondition());
    p.context = context_msg;

    return p;
}


float ManagedPlan::computeDeadline(const vector<PlanItem>& planitems)
{
    float deadline = 0.0f;
    for(PlanItem pi : planitems)
        deadline += pi.duration;
    return deadline;
}

// overload `==` operator 
bool operator==(ManagedPlan const &mp1, ManagedPlan const &mp2){
     // first check based on target desire
    if(!(mp1.getDesire() == mp2.getDesire()))
        return false;
    
    vector<PlanItem> mp1_body = mp1.getBody();
    vector<PlanItem> mp2_body = mp2.getBody();

    // check based # of mg. PlanItem in its body 
    if(mp1_body.size() != mp2_body.size())
        return false;

    // check in order mg. PlanItem by  mg. PlanItem
    for(int i=0; i<mp1_body.size(); i++)
        if(mp1_body[i].action != mp2_body[i].action)
            return false;
    
    vector<ManagedCondition> mp1_precondition = mp1.getPrecondition();
    vector<ManagedCondition> mp2_precondition = mp2.getPrecondition();

    vector<ManagedCondition> mp1_context = mp1.getContext();
    vector<ManagedCondition> mp2_context = mp2.getContext();

    // check based # of mg. condition(s) in preconditions and context conditions 
    if(mp1_precondition.size() < mp2_precondition.size() || mp1_context.size() < mp2_context.size())
        return true;

    // order do not count, so put them into four sets
    set<ManagedCondition> mp1_preconditionS;
    for(ManagedCondition mc : mp1_precondition)
        mp1_preconditionS.insert(mc);
    set<ManagedCondition> mp1_contextS;
    for(ManagedCondition mc : mp1_context)
        mp1_contextS.insert(mc);

    set<ManagedCondition> mp2_preconditionS;
    for(ManagedCondition mc : mp2_precondition)
        mp2_preconditionS.insert(mc);
    set<ManagedCondition> mp2_contextS;
    for(ManagedCondition mc : mp2_context)
        mp2_contextS.insert(mc);

    // check for every mg condition of one mg. desire if it is contained also in the other mg. desire
    for(ManagedCondition mc1 : mp1_preconditionS)
        if(mp2_preconditionS.count(mc1)==0)
            return false;

    // check for every mg condition of one mg. desire if it is contained also in the other mg. desire
    for(ManagedCondition mc1 : mp1_contextS)
        if(mp2_contextS.count(mc1)==0)
            return false;

    //otherwise equals
    return true;
}

std::ostream& operator<<(std::ostream& os, const ManagedPlan& mp)
{   
    os << "PLAN\nDesire: " << mp.getDesire();
    
    os << "\n\nActions to be performed:\n";
    for(PlanItem pi : mp.getBody())
        os << pi.action << " - " << pi.time << "\n";
    
    os << "\n\nPreconditions to be checked BEFORE execution:\n";
    for(ManagedCondition prec : mp.getPrecondition())
        os << prec << "\n";
    
    os << "\n\nContext conditions to be checked DURING execution:\n";
    for(ManagedCondition cont : mp.getContext())
        os << cont << "\n";
    
    os << "\n\nDeadline: " << mp.getPlanDeadline();
    return os;
}