#include "ros2_bdi_utils/ManagedPlan.hpp"

//#include "plansys2_domain_expert/DomainExpertClient.hpp"
//#include "plansys2_msgs/msg/Action"

//using plansys2::DomainExpertClient;
//using plansys2_msgs::msg::Action;

#define NO_PLAN "NO_PLAN"

ManagedPlan::ManagedPlan():
    body_(vector<PlanItem>()),
    precondition_(vector<ManagedBelief>()),
    context_(vector<ManagedBelief>()),
    plan_deadline_(0.0f)
    {
        Desire d = Desire{};
        d.name = NO_PLAN;
        desire_ = d;
    }


ManagedPlan::ManagedPlan(const ManagedDesire& md, const vector<PlanItem>& planitems):
    desire_(md),
    body_(planitems)
    {   
        // TODO
        // precondition_ = beliefs of first action picked
        // context_ = OR of over_all condition along all actions picked
        precondition_ = computePrecondition(planitems);
        context_ = computeContext(planitems);
        
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

    /*
    TO DO build ManagedCondition
    vector<Belief> preconditions_msg = vector<Belief>();
    for(ManagedBelief mb : precondition_)
        preconditions_msg.push_back(mb.toBelief());
    p.precondition = preconditions_msg;

    vector<Belief> context_msg = vector<Belief>();
    for(ManagedBelief mb : context_)
        context_msg.push_back(mb.toBelief());
    p.context = context_msg;
    */
    return p;
}

/*
    before the first version, just empty
    
    for a first version...
    Compute it as the precondition of the first action picked in the plan
*/
vector<ManagedBelief> ManagedPlan::computePrecondition(const vector<PlanItem>& planitems)
{
    //std::shared_ptr<DomainExpertClient> domain_expert = std::make_shared<plansys2::DomainExpertClient>();

    vector<ManagedBelief> precondition = vector<ManagedBelief>();
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
vector<ManagedBelief> ManagedPlan::computeContext(const vector<PlanItem>& planitems)
{
    vector<ManagedBelief> context = vector<ManagedBelief>();
    return context;
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
    
    vector<ManagedBelief> mp1_precondition = mp1.getPrecondition();
    vector<ManagedBelief> mp2_precondition = mp2.getPrecondition();

    // check based # of mg. Belief in its preconditions 
    if(mp1_precondition.size() != mp2_precondition.size())
        return false;

    // check in order precondition belief by precondition belief
    for(int i=0; i<mp1_precondition.size(); i++)
        if(!(mp1_precondition[i] == mp2_precondition[i]))
            return false;
    
    vector<ManagedBelief> mp1_context = mp1.getContext();
    vector<ManagedBelief> mp2_context = mp2.getContext();

    // check based # of mg. Belief in its context 
    if(mp1_context.size() != mp2_context.size())
        return false;

    // check in order context condition by context condition
    for(int i=0; i<mp1_context.size(); i++)
        if(!(mp1_context[i] == mp2_context[i]))
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
    for(ManagedBelief prec : mp.getPrecondition())
        os << prec << "\n";
    
    os << "\n\nContext conditions to be checked DURING execution:\n";
    for(ManagedBelief cont : mp.getContext())
        os << cont << "\n";
    
    os << "\n\nDeadline: " << mp.getPlanDeadline();
    return os;
}