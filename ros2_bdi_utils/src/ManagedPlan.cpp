#include "ros2_bdi_utils/ManagedPlan.hpp"

#include "ros2_bdi_interfaces/msg/desire.hpp"

using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::BDIPlan;

using BDIManaged::ManagedPlan;

using std::vector;

#define NO_PLAN "NO_PLAN"

ManagedPlan::ManagedPlan():
    body_(vector<PlanItem>()),
    precondition_(ManagedConditionsDNF{}),
    context_(ManagedConditionsDNF{}),
    plan_deadline_(0.0f)
    {
        Desire d = Desire{};
        d.name = NO_PLAN;
        target_ = d;
    }


ManagedPlan::ManagedPlan(const ManagedDesire& md, const vector<PlanItem>& planitems):
    target_(md),
    body_(planitems),
    precondition_(ManagedConditionsDNF{}),
    context_(ManagedConditionsDNF{})
    {   
        plan_deadline_ = computeDeadline(planitems);
    }

ManagedPlan::ManagedPlan(const ManagedDesire& md, const vector<PlanItem>& planitems, 
    const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context):
    target_(md),
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
    p.target = target_.toDesire();
    p.actions = body_;

    p.precondition = precondition_.toConditionsDNF();
    p.context = context_.toConditionsDNF();

    return p;
}


float ManagedPlan::computeDeadline(const vector<PlanItem>& planitems)
{
    float deadline = 0.0f;
    // you cannot compute the sum of all duration, because not all plans are 
    // linear sequence of actions (i.e. actions can start in group and/or actions
    // can start when other actions during plan exec. has not finished yet)
    for(PlanItem pi : planitems)
        deadline = std::max(deadline, pi.time + pi.duration);
    return deadline;
}

// overload `==` operator 
bool BDIManaged::operator==(ManagedPlan const &mp1, ManagedPlan const &mp2){
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

std::ostream& BDIManaged::operator<<(std::ostream& os, const ManagedPlan& mp)
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