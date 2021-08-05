#include "ros2_bdi_utils/ManagedPlan.hpp"

//#include "plansys2_domain_expert/DomainExpertClient.hpp"
//#include "plansys2_msgs/msg/Action"

//using plansys2::DomainExpertClient;
//using plansys2_msgs::msg::Action;

#define NO_PLAN "NO_PLAN"

ManagedPlan::ManagedPlan():
    body_(vector<PlanItem>()),
    precondition_(ManagedConditionsDNF{}),
    context_(ManagedConditionsDNF{}),
    plan_deadline_(0.0f)
    {
        Desire d = Desire{};
        d.name = NO_PLAN;
        desire_ = d;
    }


ManagedPlan::ManagedPlan(const ManagedDesire& md, const vector<PlanItem>& planitems):
    desire_(md),
    body_(planitems),
    precondition_(ManagedConditionsDNF{}),
    context_(ManagedConditionsDNF{})
    {   
        plan_deadline_ = computeDeadline(planitems);
    }

ManagedPlan::ManagedPlan(const ManagedDesire& md, const vector<PlanItem>& planitems, 
    const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context):
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

    p.precondition = precondition_.toConditionsDNF();
    p.context = context_.toConditionsDNF();

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

    //otherwise equals
    return mp1.getPrecondition() == mp2.getPrecondition() && mp1.getContext() == mp2.getContext();
}

std::ostream& operator<<(std::ostream& os, const ManagedPlan& mp)
{   
    os << "PLAN\nDesire: " << mp.getDesire();
    
    os << "\n\nActions to be performed:\n";
    for(PlanItem pi : mp.getBody())
        os << pi.action << " - " << pi.time << "\n";
    
    os << "\n\nPreconditions to be checked BEFORE execution:\n";
    os << mp.getPrecondition() << "\n";
    
    os << "\n\nContext conditions to be checked DURING execution:\n";
    os << mp.getContext() << "\n";
    
    os << "\n\nDeadline: " << mp.getPlanDeadline();
    return os;
}