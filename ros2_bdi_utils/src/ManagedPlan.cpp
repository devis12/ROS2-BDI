#include "ros2_bdi_utils/ManagedPlan.hpp"

//#include "plansys2_domain_expert/DomainExpertClient.hpp"
//#include "plansys2_msgs/msg/Action"

//using plansys2::DomainExpertClient;
//using plansys2_msgs::msg::Action;

ManagedPlan::ManagedPlan():
    desire_(Desire{}),
    body_(vector<PlanItem>()),
    precondition_(vector<ManagedBelief>()),
    context_(vector<ManagedBelief>()),
    plan_deadline_(0.0f)
    {}


ManagedPlan::ManagedPlan(const ManagedDesire& md, const Plan& plan):
    desire_(md),
    body_(plan.items)
    {   
        // precondition_ = beliefs of first action picked
        // context_ = OR of over_all condition along all actions picked
        plan_deadline_ = computeDeadline(plan);
    }

Plan ManagedPlan::toPlan() const
{
    Plan p = Plan();
    p.items = body_;
    return p;
}

/*
    before the first version, just empty
    
    for a first version...
    Compute it as the precondition of the first action picked in the plan
*/
vector<Belief> ManagedPlan::computePrecondition(const Plan& plan)
{
    //std::shared_ptr<DomainExpertClient> domain_expert = std::make_shared<plansys2::DomainExpertClient>();

    vector<Belief> precondition = vector<Belief>();
    /*
    PlanItem first_action = plan.items[0];
    Action pddlAction = domain_expert->getAction(first_action.action);
    if(pddlAction.name != "")
    {
        for(auto pddlPrecondition : pddlAction.preconditions)
        {
            pddlPrecondition.
        }
    }
    */
    return precondition;
}

/*
    before the first version, just empty
    
    for a first version...
    Compute it as the disjunction of the over all conditions of actions picked in the plan
*/
vector<Belief> ManagedPlan::computeContext(const Plan& plan)
{
    vector<Belief> context = vector<Belief>();
    return context;
}

float ManagedPlan::computeDeadline(const Plan& plan)
{
    float deadline = 0.0f;
    for(PlanItem pi : plan.items)
        deadline += pi.duration;
    return deadline;
}

std::ostream& operator<<(std::ostream& os, const ManagedPlan& mp)
{
    return os;
}