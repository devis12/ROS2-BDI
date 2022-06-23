// header file for Scheduler node
#include "ros2_bdi_core/scheduler.hpp"   
// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Scheduler node
#include "ros2_bdi_core/params/scheduler_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node (for belief set topic)
#include "ros2_bdi_core/params/belief_manager_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node (for plan exec srv & topic)
#include "ros2_bdi_core/params/plan_director_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for PlanSys2 Monitor node (for psys2 state topic)
#include "ros2_bdi_core/params/plansys_monitor_params.hpp"


#include <yaml-cpp/exceptions.h>

#include "ros2_bdi_utils/BDIFilter.hpp"
#include "ros2_bdi_utils/BDIPDDLConverter.hpp"
#include "ros2_bdi_utils/BDIYAMLParser.hpp"

#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedConditionsConjunction.hpp"
#include "ros2_bdi_utils/ManagedConditionsDNF.hpp"

using std::string;
using std::vector;
using std::set;
using std::map;
using std::mutex;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::optional;

using plansys2::DomainExpertClient;
using plansys2::ProblemExpertClient;
using plansys2::PlannerClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Goal;
using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

using ros2_bdi_interfaces::msg::LifecycleStatus;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::DesireSet;
using ros2_bdi_interfaces::msg::Condition;
using ros2_bdi_interfaces::msg::ConditionsConjunction;
using ros2_bdi_interfaces::msg::ConditionsDNF;
using ros2_bdi_interfaces::msg::PlanningSystemState;
using ros2_bdi_interfaces::msg::BDIActionExecutionInfo;
using ros2_bdi_interfaces::msg::BDIPlanExecutionInfo;
using ros2_bdi_interfaces::srv::BDIPlanExecution;

using BDIManaged::ManagedBelief;
using BDIManaged::ManagedDesire;
using BDIManaged::ManagedPlan;

Scheduler::Scheduler()
  : rclcpp::Node(SCHEDULER_NODE_NAME), state_(STARTING)
{
    psys2_comm_errors_ = 0;
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);
    this->declare_parameter(PARAM_MAX_TRIES_COMP_PLAN, 8);
    this->declare_parameter(PARAM_MAX_TRIES_EXEC_PLAN, 8);
    this->declare_parameter(PARAM_RESCHEDULE_POLICY, VAL_RESCHEDULE_POLICY_NO_IF_EXEC);
    this->declare_parameter(PARAM_AUTOSUBMIT_PREC, false);
    this->declare_parameter(PARAM_AUTOSUBMIT_CONTEXT, false);
    this->declare_parameter(PARAM_PLANNING_MODE, PLANNING_MODE_OFFLINE);

    sel_planning_mode_ = this->get_parameter(PARAM_PLANNING_MODE).as_string() == PLANNING_MODE_OFFLINE? OFFLINE : ONLINE;
    this->undeclare_parameter(PARAM_PLANNING_MODE);
}

/*
    Init to call at the start, after construction method, to get the node actually started
    initialing planner client instance, 
    retrieving agent_id_ (thus namespace)
    defining work timer,
    belief set subscriber callback,
    desire set publisher,
    add/del desire subscribers callback
*/
void Scheduler::init()
{ 
    //agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    // initializing domain expert
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    // initializing problem expert
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();

    // initializing planner client
    planner_client_ = std::make_shared<plansys2::PlannerClient>();

    // Declare empty desire set
    desire_set_ = set<ManagedDesire>();
    // wait for it to be init
    init_dset_ = false;

    //Desire set publisher
    desire_set_publisher_ = this->create_publisher<DesireSet>(DESIRE_SET_TOPIC, 10);

    rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
    qos_keep_all.keep_all();

    //lifecycle status init
    auto lifecycle_status = LifecycleStatus{};
    lifecycle_status_ = map<string, uint8_t>();
    lifecycle_status_[BELIEF_MANAGER_NODE_NAME] = lifecycle_status.BOOTING;
    lifecycle_status_[SCHEDULER_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[PLAN_DIRECTOR_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[PSYS_MONITOR_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[EVENT_LISTENER_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[MA_REQUEST_HANDLER_NODE_NAME] = lifecycle_status.UNKNOWN;

    //Lifecycle status publisher
    lifecycle_status_publisher_ = this->create_publisher<LifecycleStatus>(LIFECYCLE_STATUS_TOPIC, 10);

    //Lifecycle status subscriber
    lifecycle_status_subscriber_ = this->create_subscription<LifecycleStatus>(
                LIFECYCLE_STATUS_TOPIC, qos_keep_all,
                bind(&Scheduler::callbackLifecycleStatus, this, _1));

    //Check for plansys2 active state flags init to false
    psys2_planner_active_ = false;
    psys2_domain_expert_active_ = false;
    psys2_problem_expert_active_ = false;
    //plansys2 nodes status subscriber (receive notification from plansys2_monitor node)
    plansys2_status_subscriber_ = this->create_subscription<PlanningSystemState>(
                PSYS_STATE_TOPIC, qos_keep_all,
                bind(&Scheduler::callbackPsys2State, this, _1));

    //Desire to be added notification
    add_desire_subscriber_ = this->create_subscription<Desire>(
                ADD_DESIRE_TOPIC, qos_keep_all,
                bind(&Scheduler::addDesireTopicCallBack, this, _1));

    //Desire to be removed notification
    del_desire_subscriber_ = this->create_subscription<Desire>(
                DEL_DESIRE_TOPIC, qos_keep_all,
                bind(&Scheduler::delDesireTopicCallBack, this, _1));

    //belief_set_subscriber_ 
    belief_set_subscriber_ = this->create_subscription<BeliefSet>(
                BELIEF_SET_TOPIC, qos_keep_all,
                bind(&Scheduler::updatedBeliefSet, this, _1));

    plan_exec_srv_client_ = std::make_shared<TriggerPlanClient>(PLAN_EXECUTION_SRV + string("_s_caller"));

    plan_exec_info_subscriber_ = this->create_subscription<BDIPlanExecutionInfo>(
        PLAN_EXECUTION_TOPIC, 10,
        bind(&Scheduler::updatePlanExecution, this, _1)
    );

    //loop to be called regularly to perform work (publish belief_set_, sync with plansys2 problem_expert node...)
    do_work_timer_ = this->create_wall_timer(
        milliseconds(500),
        bind(&Scheduler::step, this));

    RCLCPP_INFO(this->get_logger(), "Scheduler node initialized");
}
  
/*
    Main loop of work called regularly through a wall timer
*/
void Scheduler::step()
{
    // all psys2 up -> no psys2 comm. errors
    if(psys2_planner_active_ && psys2_domain_expert_active_ && psys2_problem_expert_active_ )
        psys2_comm_errors_ = 0;

    //if psys2 appears crashed, crash too
    if(psys2_comm_errors_ > MAX_COMM_ERRORS)
        rclcpp::shutdown();

    if(step_counter_ % 4 == 0)
        lifecycle_status_publisher_->publish(getLifecycleStatus());

    switch (state_) {
        
        case STARTING:
        {
            if(psys2_domain_expert_active_ && psys2_problem_expert_active_){
                psys2_comm_errors_ = 0;
                if(!init_dset_)//hasn't ben tried to init desire set yet    
                {    
                    tryInitDesireSet();
                    init_dset_ = true;
                }
                
                RCLCPP_INFO(this->get_logger(),"Belief manager  active = %d and plan director active = %d", 
                    lifecycle_status_[BELIEF_MANAGER_NODE_NAME], lifecycle_status_[PLAN_DIRECTOR_NODE_NAME]);
                    
                if(lifecycle_status_[BELIEF_MANAGER_NODE_NAME] == LifecycleStatus{}.RUNNING && lifecycle_status_[PLAN_DIRECTOR_NODE_NAME] == LifecycleStatus{}.RUNNING)
                {
                    //belief manager and plan director are booted and ready to go!!!
                    if((sel_planning_mode_ == OFFLINE && psys2_planner_active_) || (sel_planning_mode_ == ONLINE && javaff_planner_active_))
                    {    
                        setState(SCHEDULING);//corresponding planner is active too, so you can jump to scheduling state
                        lifecycle_status_publisher_->publish(getLifecycleStatus());
                    }
                }
            }else{
                
                if(sel_planning_mode_ == OFFLINE && !psys2_planner_active_)
                    RCLCPP_ERROR(this->get_logger(), "PlanSys2 Planner still not active");

                if(sel_planning_mode_ == ONLINE && !javaff_planner_active_)
                    RCLCPP_ERROR(this->get_logger(), "JavaFF Online Planner still not active");

                if(!psys2_domain_expert_active_)
                    RCLCPP_ERROR(this->get_logger(), "PlanSys2 Domain Expert still not active");

                if(!psys2_problem_expert_active_)
                    RCLCPP_ERROR(this->get_logger(), "PlanSys2 Problem Expert still not active");


                psys2_comm_errors_++;
            }

            break;
        }

        case SCHEDULING:
        {   
            publishDesireSet();

            auto reschedulePolicy = this->get_parameter(PARAM_RESCHEDULE_POLICY).as_string();
            /*
                Either the reschedule policy is no if a plan is executing AND there is no plan currently in exec
                or the reschedule policy allows rescheduling while plan is in exec
            */
            if(reschedulePolicy == VAL_RESCHEDULE_POLICY_NO_IF_EXEC && noPlanExecuting() 
                || reschedulePolicy != VAL_RESCHEDULE_POLICY_NO_IF_EXEC)
            {
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Reschedule to select new plan to be executed");

                reschedule();
            }
        }

        case PAUSE:
        {   
            //RCLCPP_INFO(this->get_logger(), "Not the moment to ask for new cleaning tasks yet");    
            break;
        }
        default:
            break;
    }

    step_counter_++;
}

/*Build updated LifecycleStatus msg*/
LifecycleStatus Scheduler::getLifecycleStatus()
{
    LifecycleStatus lifecycle_status = LifecycleStatus{};
    lifecycle_status.node_name = SCHEDULER_NODE_NAME;
    lifecycle_status.status = (state_ == STARTING)? lifecycle_status.BOOTING : lifecycle_status.RUNNING;
    return lifecycle_status;
}


/*
    Publish the current desire set of the agent in agent_id_/desire_set topic
*/
void Scheduler::publishDesireSet()
{
    DesireSet dset_msg = BDIFilter::extractDesireSetMsg(desire_set_);
    dset_msg.agent_id = agent_id_;
    desire_set_publisher_->publish(dset_msg);
}

/*
    Received notification about PlanSys2 nodes state by plansys2 monitor node
*/
void Scheduler::callbackPsys2State(const PlanningSystemState::SharedPtr msg)
{
    psys2_problem_expert_active_ = msg->problem_expert_active;
    psys2_domain_expert_active_ = msg->domain_expert_active;
    psys2_planner_active_ = msg->offline_planner_active;
    javaff_planner_active_ = msg->online_planner_active;
}

/*
    Expect to find yaml file to init the desire set in "/tmp/{agent_id}/init_dset.yaml"
*/
void Scheduler::tryInitDesireSet()
{
    string init_dset_filepath = "/tmp/"+this->get_parameter(PARAM_AGENT_ID).as_string() + "/" + INIT_DESIRE_SET_FILENAME; 
    try{
        vector<ManagedDesire> init_mgdesires = BDIYAMLParser::extractMGDesires(init_dset_filepath);
        for(ManagedDesire initMGDesire : init_mgdesires)
            if(initMGDesire.getValue().size() > 0)
                    addDesire(initMGDesire);
        if(this->get_parameter(PARAM_DEBUG).as_bool())
            RCLCPP_INFO(this->get_logger(), "Desire set initialization performed through " + init_dset_filepath);
    
    }catch(const YAML::BadFile& bfile){
        RCLCPP_ERROR(this->get_logger(), "Bad File: Desire set initialization failed because init. file " + init_dset_filepath + " hasn't been found");
    }catch(const YAML::ParserException& bpars){
        RCLCPP_ERROR(this->get_logger(), "YAML Parser Exception: Desire set initialization failed because init. file " + init_dset_filepath + " doesn't present a valid YAML format");
    }catch(const YAML::BadConversion& bconvfile){
        RCLCPP_ERROR(this->get_logger(), "Bad Conversion: Desire set initialization failed because init. file " + init_dset_filepath + " doesn't present a valid desire array");
    }catch(const YAML::InvalidNode& invalid_node){
        RCLCPP_ERROR(this->get_logger(), "Invalid Node: Desire set initialization failed because init. file " + init_dset_filepath + " doesn't present a valid desire array");
    }   
}

/*
    returns ACCEPTED iff managed belief can be put as part of a desire's value
    wrt. to its syntax
*/
TargetBeliefAcceptance Scheduler::targetBeliefAcceptanceCheck(const ManagedBelief& mb)
{
    if(mb.pddlType() != Belief().PREDICATE_TYPE)//not predicate -> not accepted
            return UNKNOWN_PREDICATE;

    optional<Predicate> optPredDef = domain_expert_->getPredicate(mb.getName());
    if(!optPredDef.has_value())//incorrect predicate name
        return UNKNOWN_PREDICATE;

    Predicate predDef = optPredDef.value();
    if(predDef.parameters.size() != mb.getParams().size())
        return SYNTAX_ERROR;
        
    vector<string> params = mb.getParams();
    for(int i=0; i<params.size(); i++)
    {
        string instanceName = params[i];
        optional<Instance> opt_ins = problem_expert_->getInstance(instanceName);
        
        if(!opt_ins.has_value())//found a not valid instance in one of the goal predicates       
            return UNKNOWN_INSTANCES;
        else if(opt_ins.value().type != predDef.parameters[i].type && //instance types not matching definition
            std::find(predDef.parameters[i].sub_types.begin(), predDef.parameters[i].sub_types.end(), opt_ins.value().type) == predDef.parameters[i].sub_types.end()) //failing also checks within sub types
            return UNKNOWN_INSTANCES;
    }

    return ACCEPTED;
}

/*
        Check with the domain_expert and problem_expert to understand if this is a valid goal
    (i.e. valid predicates and valid instances defined within them)

    returns true iff managed desire can be considered syntactically correct
    (no syntactically incorrect beliefs)
*/
TargetBeliefAcceptance Scheduler::desireAcceptanceCheck(const ManagedDesire& md)
{
    for(ManagedBelief mb : md.getValue())
    {
        auto acceptance = targetBeliefAcceptanceCheck(mb);
        if(acceptance != ACCEPTED)
            return acceptance;
    }

    return ACCEPTED;
}

/*
    Launch execution of selectedPlan; if successful current plan is pushed into waiting plans for execution gets value of selectedPlan
    return true if successful
*/
bool Scheduler::launchPlanExecution(const BDIManaged::ManagedPlan& selectedPlan)
{   
    //trigger plan execution
    bool triggered = plan_exec_srv_client_->triggerPlanExecution(selectedPlan.toPlan());
    if(triggered)
        current_plan_ = selectedPlan;// selectedPlan can now be set as currently executing plan

    if(this->get_parameter(PARAM_DEBUG).as_bool())
    {
        if(triggered) RCLCPP_INFO(this->get_logger(), "Triggered new plan execution fulfilling desire \"" + current_plan_.getDesire().getName() + "\" success");
        else RCLCPP_INFO(this->get_logger(), "Triggered new plan execution fulfilling desire \"" + selectedPlan.getDesire().getName() + "\" failed");
    }

    return triggered;
}

/*
    Abort execution of current plan; if successful waiting plans is popped empty
    return true if successful
*/
bool Scheduler::abortCurrentPlanExecution()
{
    bool aborted = plan_exec_srv_client_->abortPlanExecution(current_plan_.toPlan());
    if(aborted)
    {
        if(this->get_parameter(PARAM_DEBUG).as_bool())
            RCLCPP_INFO(this->get_logger(), "Aborted plan execution fulfilling desire \"%s\"", current_plan_.getDesire().getName());
        
        current_plan_ = BDIManaged::ManagedPlan{}; //no plan in execution
    }  
    return aborted;
}


/*
    If selected plan fit the minimal requirements for a plan (i.e. not empty body and a desire which is in the desire_set)
    try triggering its execution by srv request to PlanDirector (/{agent}/plan_execution) by exploiting the TriggerPlanClient

*/
bool Scheduler::tryTriggerPlanExecution(const ManagedPlan& selectedPlan)
{      
    string reschedulePolicy = this->get_parameter(PARAM_RESCHEDULE_POLICY).as_string();
    bool noPlan = noPlanExecuting();
    //rescheduling not ammitted -> a plan already executing and policy not admit any switch with higher priority plans
    if(reschedulePolicy == VAL_RESCHEDULE_POLICY_NO_IF_EXEC && !noPlan)
        return false;

    //rescheduling possible, but plan currently in exec (substitute just for plan with higher priority)
    bool planinExec = reschedulePolicy != VAL_RESCHEDULE_POLICY_NO_IF_EXEC && !noPlan;
    if(planinExec)
    {
        //before triggering new plan, abort the one currently in exec
        if(this->get_parameter(PARAM_DEBUG).as_bool())
                RCLCPP_INFO(this->get_logger(), "Ready to abort plan for desire \"" + current_plan_.getDesire().getName() + "\"" + 
                            " in order to trigger plan execution for desire \"" + selectedPlan.getDesire().getName() + "\"");
            
        //trigger plan abortion
        if(!abortCurrentPlanExecution())
            return false;//current plan abortion failed
    }
    
    //desire still in desire set
    bool desireInDesireSet = desire_set_.count(selectedPlan.getDesire())==1;
    
    //check that a proper plan has been selected (with actions and fulfilling a desire in the desire_set_)
    if(selectedPlan.getActionsExecInfo().size() == 0 || !desireInDesireSet)
        return false;
    

    return launchPlanExecution(selectedPlan);
}

/*
    Given the current knowledge of the belief set, decide if a given desire
    is already fulfilled
*/
bool Scheduler::isDesireSatisfied(ManagedDesire& md)
{
    return md.isFulfilled(belief_set_);
}

/*
    The belief set has been updated
*/
void Scheduler::updatedBeliefSet(const BeliefSet::SharedPtr msg)
{
    set<ManagedBelief> newBeliefSet = BDIFilter::extractMGBeliefs(msg->value);
    bool bsetModified = false;//is belief set altered from last update?
    
    for(ManagedBelief mb : newBeliefSet)
        if(belief_set_.count(mb) == 0)//check if new belief set has new items
        {
            bsetModified = true;
            break;
        }
    
    if(!bsetModified)
        for(ManagedBelief mb : belief_set_)
            if(newBeliefSet.count(mb) == 0)//check if new belief set has lost a few items
            {
                bsetModified = true;
                break;
            }

    if(bsetModified)//if belief set appears different from last update
    {
        belief_set_ = newBeliefSet;//update current mirroring of the belief set
        
        checkForSatisfiedDesires();//check for satisfied desires
        if(state_ == SCHEDULING)
            reschedule();//do a rescheduling
    }
    
}

/*  
    Someone has publish a new desire to be fulfilled in the respective topic
*/
void Scheduler::addDesireTopicCallBack(const Desire::SharedPtr msg)
{   
    ManagedDesire mdAdd = ManagedDesire{(*msg)};
    bool added = addDesire(mdAdd);

    if(added)//addition done
    {   
        publishDesireSet();

        //call specific methods of SchedulerOffline/SchedulerOnline
        postAddDesireSuccess(mdAdd);
    }
}

/*  
    Someone has publish a desire to be removed from the one to be fulfilled (if present)
    in the respective topic
*/
void Scheduler::delDesireTopicCallBack(const Desire::SharedPtr msg)
{
    ManagedDesire mdDel = ManagedDesire{(*msg)};
    bool deleted = delDesire(mdDel);

    if(deleted)
    {
        publishDesireSet();
        
         //call specific methods of SchedulerOffline/SchedulerOnline
        postDelDesireSuccess(mdDel);
    }
}

/*
    Add desire Critical Section (to be executed AFTER having acquired mtx_add_del_.lock())

    Add desire @mdAdd to desire_set_ (if there is not yet)
    add counters for invalid desire and aborted plan in respective maps (both init to zero)
    @necessaryForMd is another ManagedDesire which can be present if @mdAdd is necessary 
    for the fulfillment of it (e.g. @mdAdd is derived from preconditions or context)
    @necessaryForMd value (if exists) has to be already in the desire_set_
*/
bool Scheduler::addDesireCS(ManagedDesire mdAdd, optional<ManagedDesire> necessaryForMD, const string& suffixDesireGroup)
{
    if(mtx_add_del_.try_lock())
    {   
        //if acquired, it means we were not in CS, release it and return false -> operation not valid
        mtx_add_del_.unlock();
        return false;
    }
    
    bool added = false;
    if(computed_plan_desire_map_.count(mdAdd.getName())==0 && aborted_plan_desire_map_.count(mdAdd.getName())==0 && 
            desire_set_.count(mdAdd)==0)//desire already there (or diff. desire but with same name identifier)
    {
        if(necessaryForMD.has_value() && desire_set_.count(necessaryForMD.value())==1)
        {
            // @mdAdd linked to another MGdesire for which it is necessary in order to grant its execution
            // put as @mdAdd's group mdAdd.name + suffix (suffix can be "_precondition"/"_context")
            mdAdd.setDesireGroup(necessaryForMD.value().getName() + suffixDesireGroup);
        }

        desire_set_.insert(mdAdd);
        computed_plan_desire_map_.insert(std::pair<string, int>(mdAdd.getName(), 0));//to count invalid goal computations and discard after x
        aborted_plan_desire_map_.insert(std::pair<string, int>(mdAdd.getName(), 0));//to count invalid goal computations and discard after x
        
        added = true;
    }

    return added;
}

/*
    Del desire from desire_set CRITICAL SECTION (to be called after having acquired mtx_add_del_ lock)
    In Addition Deleting atomically all the desires within the same desire group
    if @wipeSameGroup equals to true
*/
bool Scheduler::delDesireCS(const ManagedDesire mdDel, const bool& wipeSameGroup)
{
    if(mtx_add_del_.try_lock())
    {   
        //if acquired, it means we were not in CS, release it and return false -> operation not valid
        mtx_add_del_.unlock();
        return false;
    }

    bool deleted = false;
    //erase values from desires map
    if(computed_plan_desire_map_.count(mdDel.getName()) > 0)
        computed_plan_desire_map_.erase(mdDel.getName());
    if(aborted_plan_desire_map_.count(mdDel.getName()) > 0)
        aborted_plan_desire_map_.erase(mdDel.getName());

    if(desire_set_.count(mdDel)!=0)
    {
        desire_set_.erase(desire_set_.find(mdDel));
        computed_plan_desire_map_.erase(mdDel.getName());
        deleted = true;
        //RCLCPP_INFO(this->get_logger(), "Desire \"" + mdDel.getName() + "\" removed!");
    }/*else
        RCLCPP_INFO(this->get_logger(), "Desire \"" + mdDel.getName() + "\" to be removed NOT found!");*/
    
    if(wipeSameGroup && mdDel.hasParent()) // desire being part of a group where you need to satisfy just one
        delDesireInGroupCS(mdDel.getDesireGroup()); // delete others in the same group

    return deleted;
}

/*
    Del desire group from desire_set CRITICAL SECTION (to be called after having acquired mtx_add_del_ lock)
    Deleting atomically all the desires within the same desire group
*/
void Scheduler::delDesireInGroupCS(const string& desireGroup)
{
    vector<ManagedDesire> toBeDiscarded;

    for(ManagedDesire md : desire_set_)
        if(md.getDesireGroup() == desireGroup)
            toBeDiscarded.push_back(md);
    
    for(ManagedDesire md : toBeDiscarded)
        delDesireCS(md, false);//you're already iterating over all the desires within the same group
}