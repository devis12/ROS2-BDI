// header file for Plan Director node
#include "ros2_bdi_core/plan_director.hpp"
// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Plan Director node
#include "ros2_bdi_core/params/plan_director_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for PlanSys2 Monitor node (for topics' names)
#include "ros2_bdi_core/params/belief_manager_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Scheduler 
#include "ros2_bdi_core/params/scheduler_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for PlanSys2 Monitor node (for psys2 state topic)
#include "ros2_bdi_core/params/plansys_monitor_params.hpp"

#include <boost/algorithm/string.hpp>

#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "ros2_bdi_utils/ManagedConditionsDNF.hpp"
#include "ros2_bdi_utils/PDDLBDIConverter.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"
#include "ros2_bdi_utils/PDDLUtils.hpp"


using std::string;
using std::set;
using std::map;
using std::vector;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::chrono::high_resolution_clock;
using std::bind;
using std::placeholders::_1;
using std::placeholders::_2;
using std::optional;

using plansys2::DomainExpertClient;
using plansys2::ProblemExpertClient;
using plansys2::ExecutorClient;
using plansys2_msgs::msg::ActionExecutionInfo;
using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;
using plansys2_msgs::msg::Action;
using plansys2_msgs::msg::DurativeAction;

using ros2_bdi_interfaces::msg::LifecycleStatus;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::PlanningSystemState;
using ros2_bdi_interfaces::msg::BDIActionExecutionInfo;
using ros2_bdi_interfaces::msg::BDIPlanExecutionInfo;
using ros2_bdi_interfaces::msg::BDIPlan;

using ros2_bdi_interfaces::srv::BDIPlanExecution;

using BDIManaged::ManagedBelief;
using BDIManaged::ManagedConditionsDNF;
using BDIManaged::ManagedDesire;
using BDIManaged::ManagedPlan;

PlanDirector::PlanDirector()
  : rclcpp::Node(PLAN_DIRECTOR_NODE_NAME), state_(STARTING)
{
    psys2_comm_errors_ = 0;
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);
    this->declare_parameter(PARAM_CANCEL_AFTER_DEADLINE, DEFAULT_VAL_CANCEL_AFTER_DEADLINE);
    this->declare_parameter(PARAM_PLANNING_MODE, PLANNING_MODE_OFFLINE);

    //object to notify the absence of a current plan execution
    no_plan_msg_ = BDIPlanExecutionInfo();
    no_plan_msg_.target = Desire();
    no_plan_msg_.target.name = (ManagedPlan{}).getPlanTarget().getName();
    no_plan_msg_.target.value = vector<Belief>();
    no_plan_msg_.actions_exec_info = vector<BDIActionExecutionInfo>();
    no_plan_msg_.current_time = 0.0f;
    no_plan_msg_.estimated_deadline = 0.0f;

    sel_planning_mode_ = this->get_parameter(PARAM_PLANNING_MODE).as_string() == PLANNING_MODE_OFFLINE? OFFLINE : ONLINE;
    this->undeclare_parameter(PARAM_PLANNING_MODE);
}

/*
    Init to call at the start, after construction method, to get the node actually started
    initialing psys2 executor client instance, 
    retrieving agent_id_ (thus namespace)
    defining work timer
*/
void PlanDirector::init()
{ 
    //agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    // initializing executor client for psys2
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    // initializing domain expert client for psys2
    domain_expert_client_ = std::make_shared<plansys2::DomainExpertClient>();
    // initializing problem expert client for psys2
    problem_expert_client_ = std::make_shared<plansys2::ProblemExpertClient>();

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

    // init step_counter
    step_counter_ = 0;

    //Lifecycle status publisher
    lifecycle_status_publisher_ = this->create_publisher<LifecycleStatus>(LIFECYCLE_STATUS_TOPIC, 10);

    //Lifecycle status subscriber
    lifecycle_status_subscriber_ = this->create_subscription<LifecycleStatus>(
                LIFECYCLE_STATUS_TOPIC, qos_keep_all,
                bind(&PlanDirector::callbackLifecycleStatus, this, _1));

    //Check for plansys2 active state flags init to false
    psys2_domain_expert_active_ = false;
    psys2_problem_expert_active_ = false;
    psys2_executor_active_ = false;
    //plansys2 nodes status subscriber (receive notification from plansys2_monitor node)
    plansys2_status_subscriber_ = this->create_subscription<PlanningSystemState>(
                PSYS_STATE_TOPIC, qos_keep_all,
                bind(&PlanDirector::callbackPsys2State, this, _1));

    //belief_set_subscriber_ 
    belief_set_subscriber_ = this->create_subscription<BeliefSet>(
                BELIEF_SET_TOPIC, qos_keep_all,
                bind(&PlanDirector::updatedBeliefSet, this, _1));

    // belief add + belief del publishers
    belief_add_publisher_ = this->create_publisher<Belief>(ADD_BELIEF_TOPIC, 10);
    belief_del_publisher_ = this->create_publisher<Belief>(DEL_BELIEF_TOPIC, 10);

    // init server for triggering new plan execution
    server_plan_exec_ = this->create_service<BDIPlanExecution>(PLAN_EXECUTION_SRV, 
        bind(&PlanDirector::handlePlanRequest, this, _1, _2));

    // set NO_PLAN as current_plan_ 
    setNoPlanMsg();
    // plan execution notification
    plan_exec_publisher_ = this->create_publisher<BDIPlanExecutionInfo>(PLAN_EXECUTION_TOPIC, 10);

    //loop to be called regularly to perform work (publish belief_set_, sync with plansys2 problem_expert node...)
    do_work_timer_ = this->create_wall_timer(
        milliseconds(NO_PLAN_INTERVAL),
        bind(&PlanDirector::step, this));

    RCLCPP_INFO(this->get_logger(), "Plan director node initialized");
}

/*
    Main loop of work called regularly through a wall timer
*/
void PlanDirector::step()
{
    // all psys2 up -> no psys2 comm. errors
    if( psys2_executor_active_ && psys2_domain_expert_active_ && psys2_problem_expert_active_ )
        psys2_comm_errors_ = 0;

    //if psys2 appears crashed, crash too
    if(psys2_comm_errors_ > MAX_COMM_ERRORS)
        rclcpp::shutdown();

    if(step_counter_ % 4 == 0)
        lifecycle_status_publisher_->publish(getLifecycleStatus());

    switch (state_) {
        
        case STARTING:
        {
            if(psys2_executor_active_){
                setState(READY);
                lifecycle_status_publisher_->publish(getLifecycleStatus());
            }else{
                RCLCPP_ERROR(this->get_logger(), "PlanSys2 Executor still not active");
                psys2_comm_errors_++;
            }

            break;
        }

        case READY:
        {
            if(this->get_parameter(PARAM_DEBUG).as_bool())
                RCLCPP_INFO(this->get_logger(), "Ready to accept new plan to be executed");
            publishNoPlanExec();//notify node it is currently on idle, i.e. not executing any plan
            break;
        }

        case EXECUTING:
        {    
            //RCLCPP_INFO(this->get_logger(), "Checking plan execution");
            executingPlan();
            break;
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

/*
    Check planExecution feedback from plansys2 executor + check for context conditions
*/
void PlanDirector::executingPlan()
{
    counter_check_++; 
    checkPlanExecution(); // get feedback from executor
    checkContextConditions(); // check if context conditions are still valid and true -> abort otherwise
}

/*
    When in READY state, msg to publish in plan_execution_info to notify it 
    (i.e. notify you're not executing any plan)
*/
void PlanDirector::publishNoPlanExec()
{
    auto current_plan_desire = current_plan_.getPlanTarget();
    if(current_plan_.getActionsExecInfo().size() == 0 &&
            current_plan_desire.getName() == no_plan_msg_.target.name && current_plan_desire.getPriority() == 0.0f)
    {
        //no plan currently in execution -> proceeds notifying that
        plan_exec_publisher_->publish(no_plan_msg_);
    }
}

/*Build updated LifecycleStatus msg*/
LifecycleStatus PlanDirector::getLifecycleStatus()
{
    LifecycleStatus lifecycle_status = LifecycleStatus{};
    lifecycle_status.node_name = PLAN_DIRECTOR_NODE_NAME;
    lifecycle_status.status = (state_ == STARTING)? lifecycle_status.BOOTING : lifecycle_status.RUNNING;
    return lifecycle_status;
}

/*
    Received notification about PlanSys2 nodes state by plansys2 monitor node
*/
void PlanDirector::callbackPsys2State(const PlanningSystemState::SharedPtr msg)
{
    psys2_domain_expert_active_ = msg->domain_expert_active;
    psys2_problem_expert_active_ = msg->problem_expert_active;
    psys2_executor_active_ = msg->executor_active;
}

/*
    Currently executing no plan
*/
bool PlanDirector::executingNoPlan()
{
    return state_ != EXECUTING && current_plan_.getActionsExecInfo().size() == 0 && current_plan_.getPlanTarget().getName() == ManagedPlan{}.getPlanTarget().getName();
}

/*
    Cancel current plan execution (if any) and information preserved in it
*/
void PlanDirector::cancelCurrentPlanExecution()
{
    //cancel plan execution
    executor_client_->cancel_plan_execution();
    if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_INFO(this->get_logger(), "Aborted plan execution");

    checkPlanExecution();//to publish aborting and notifying subscribers
}


/*
    Start new plan execution -> true if correctly started
*/
bool PlanDirector::startPlanExecution(const ManagedPlan& mp)
{
    // prepare plansys2 msg for plan execution
    Plan plan_to_execute = mp.toPsys2Plan();

    // select current_plan_ which will start execution
    current_plan_ = mp;
    // current_plan_start_ = high_resolution_clock::now();//plan started now
    bool started = executor_client_->start_plan_execution(plan_to_execute);

    if(started)
    {
        setState(EXECUTING);//put node in executing state
        //reset value, so they can be set at the first action execution feedback
        first_ts_plan_sec_ = -1;//reset this value
        first_ts_plan_nanosec_ = 0;//reset this value
        last_ts_plan_exec_ = -1.0f;//reset this value
        
        counter_check_ = 0;//checks performed during this plan exec

        //WORK TIMER set to PLAN EXEC MODE (callback checks more frequent)
        resetWorkTimer(PLAN_INTERVAL);

        if(this->get_parameter(PARAM_DEBUG).as_bool())
        {
            string plan_string = "";
            for(PlanItem pi : plan_to_execute.items)
                plan_string +=  std::to_string(pi.time) + "\t" + pi.action + "\t\t" + std::to_string(pi.duration) + "\n";
            RCLCPP_INFO(this->get_logger(), "Started new plan execution:\n" + plan_string + "\n");
        }
        
    }

    return started;
}

// redefine workint timer interval
void PlanDirector::resetWorkTimer(const int& ms)
{
    if(!do_work_timer_->is_canceled())
        do_work_timer_->cancel();
    
    do_work_timer_ = this->create_wall_timer(
        milliseconds(ms),
        bind(&PlanDirector::step, this));
}

/*
    Return true if plan exec request is well formed 
        - request = ABORT | EXECUTE
        - at least a planitem elem in body
        - for each plan item action
            - check its definition exists with domain expert
            - check its params are valid instances (problem expert) matching the correct types (domain expert)
*/
bool PlanDirector::validPlanRequest(const BDIPlanExecution::Request::SharedPtr request)
{
    auto req = request->request;
    if(req != request->ABORT && req != request->EXECUTE && req != request->EARLY_ABORT)//invalid request
    {
        if(this->get_parameter(PARAM_DEBUG).as_bool())
            RCLCPP_INFO(this->get_logger(), "Plan request operation = %d not valid", request->request);
        return false;
    }    

    auto plan = request->plan;
    if(plan.psys2_plan.items.size() == 0)
    {
        if(this->get_parameter(PARAM_DEBUG).as_bool())
            RCLCPP_INFO(this->get_logger(), "Plan request with empty plan");
        return false;
    }

    if(!psys2_domain_expert_active_ || !psys2_problem_expert_active_)
        psys2_comm_errors_++;
    else
    {
        for(auto planItemObj : plan.psys2_plan.items)
        {
            vector<string> actionItems = PDDLUtils::extractPlanItemActionElements(planItemObj.action);
            if(actionItems.size() == 0)
                return false;//plan item action not valid
            else
            {
                string actName = actionItems[0];//first position action name
                shared_ptr<DurativeAction> actDA = domain_expert_client_->getDurativeAction(actName);//retrieve it from the domain expert
                if(actDA->parameters.size() != actionItems.size() - 1)
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Plan request operation not valid: dur. action " + actName + " has wrong number of params");
                    return false;//plan item not valid -> unexpected num of parameters wrt domain definition of durative act
                }  
                    
                for(int i = 0 ; i<actDA->parameters.size(); i++)
                {
                    plansys2_msgs::msg::Param paramDA = actDA->parameters[i];//retrieve param domain definition
                    std::optional<plansys2::Instance> paramInstanceOpt = problem_expert_client_->getInstance(actionItems[i+1]);//retrieve corresponding parameter from plan item action
                    if(!paramInstanceOpt.has_value())
                    {
                        if(this->get_parameter(PARAM_DEBUG).as_bool())
                            RCLCPP_INFO(this->get_logger(), "Plan request operation not valid: dur. action " + actName + " presents invalid instance " + actionItems[i+1]);
                        
                        return false;//invalid instance
                    }  
                    else
                    {
                        plansys2::Instance paramInstance = paramInstanceOpt.value();
                        
                        //check type in domain == type in stated action param (check also for subtypes!!!)
                        if(paramInstance.type != paramDA.type)
                        {
                            if(std::find(paramDA.sub_types.begin(), paramDA.sub_types.end(), paramInstance.type) == paramDA.sub_types.end())//not even in the subtype array
                            {
                                if(this->get_parameter(PARAM_DEBUG).as_bool())
                                    RCLCPP_INFO(this->get_logger(), "Plan request operation not valid: dur. action " + actName + " presents invalid typed instance " + actionItems[i+1] +
                                        ": " + paramDA.type + " needed, " + paramInstance.type + " found");
                                
                                return false;//instance valid, but do not respect type of the expected param for the action
                            }
                            
                        }  
                    }    
                }
            }
            
        }
    }
    return true;
}

/*  
    Callback to handle the service request to trigger a new plan execution or abort the current one
*/
void PlanDirector::handlePlanRequest(const BDIPlanExecution::Request::SharedPtr request,
    const BDIPlanExecution::Response::SharedPtr response)
{
    if(!validPlanRequest(request))
    {
        response->success = false;
        return;
    }

    string req_action = (request->request == request->EXECUTE)? "execute" : "abort";
    RCLCPP_INFO(this->get_logger(), "Received request to " + req_action + " plan " + std::to_string(request->plan.psys2_plan.plan_index) + " fulfilling desire \"" + request->plan.target.name + "\"");
    ManagedDesire mdPlan = ManagedDesire{request->plan.target};
    ManagedConditionsDNF mdPlanPrecondition = ManagedConditionsDNF{request->plan.precondition};
    ManagedConditionsDNF mdPlanContext = ManagedConditionsDNF{request->plan.context};
    bool done = false;
    if(request->request == request->ABORT && state_ == EXECUTING)// plan requested to be aborted it's in execution
    {
        //when aborting do not check preconditions and/or context... plan executed considered equivalent regardless of that
        ManagedPlan mp_abort = ManagedPlan{request->plan.psys2_plan.plan_index, mdPlan, request->plan.psys2_plan.items};

        if(current_plan_ == mp_abort)//request to abort plan which is currently in execution
        {
            cancelCurrentPlanExecution();
            done = executingNoPlan();
        }
    }
    else if(request->request == request->EARLY_ABORT && state_ == EXECUTING)// plan requested to be aborted it's in execution
    {
        done = executor_client_->early_arrest_request(request->plan.psys2_plan);
    }
    else if(request->request == request->EXECUTE && state_ == READY)//no plan currently in exec
    {
        ManagedPlan requestedPlan = ManagedPlan{request->plan.psys2_plan.plan_index, mdPlan, request->plan.psys2_plan.items, mdPlanPrecondition, mdPlanContext};
        // verify precondition before actually trying triggering executor
        if(requestedPlan.getPrecondition().isSatisfied(belief_set_))
        {
            bool started = startPlanExecution(requestedPlan);
            done = started && state_ == EXECUTING;
            if(done)
                checkPlanExecution();// 1st checkPlanExecution for this plan (if started)
        } 
    }
        
    response->success = done;
}

/*
    Plan currently in execution, monitor and publish the feedback of its development
*/
void PlanDirector::checkContextConditions()
{
    if(!current_plan_.getContext().isSatisfied(belief_set_))
    {
        //need to abort current plan execution because context condition are not valid anymore
        if(this->get_parameter(PARAM_DEBUG).as_bool())
            RCLCPP_INFO(this->get_logger(), "Aborting current plan execution because context conditions are not satisfied");
        
        cancelCurrentPlanExecution();
    }else{
        if(counter_check_ % 4 == 0 && this->get_parameter(PARAM_DEBUG).as_bool())//print just every 4 checks
            RCLCPP_INFO(this->get_logger(), "Current plan execution can go on: at least a context condition clause is satisfied");
    }

}

/* 
    publish beliefs to be added and beliefs to be deleted as a consequence of plan abortion (rollback)
*/
void PlanDirector::publishRollbackBeliefs(const vector<Belief> rollback_belief_add, const vector<Belief> rollback_belief_del)
{
    for(Belief b_add : rollback_belief_add)
        belief_add_publisher_->publish(b_add);

    for(Belief b_del : rollback_belief_del)
        belief_del_publisher_->publish(b_del);
}

/*
    Plan currently in execution, monitor and publish the feedback of its development
*/
void PlanDirector::checkPlanExecution()
{   
    //get feedback from plansys2 api
    auto feedback = executor_client_->getFeedBack();
    BDIPlanExecutionInfo planExecutionInfo = getPlanExecutionInfo(feedback);
    current_plan_.setUpdatedInfo(planExecutionInfo); 

    plan_exec_publisher_->publish(planExecutionInfo);

    if(planExecutionInfo.status != planExecutionInfo.RUNNING)
    {
        ManagedDesire targetDes = current_plan_.getPlanTarget();
        //in any case plan execution has stopped, so go back to printing out you're not executing any plan
        resetWorkTimer(NO_PLAN_INTERVAL);
        setNoPlanMsg();
        setState(READY);

        if(planExecutionInfo.status == planExecutionInfo.ABORT /*&& !targetDes.isFulfilled(belief_set_)*/)//plan execution aborted -> beliefs rollback
            publishRollbackBeliefs(planExecutionInfo.target.rollback_belief_add, planExecutionInfo.target.rollback_belief_del);
        
        // ended run log 
        if(this->get_parameter(PARAM_DEBUG).as_bool()){
            string result_s =   ((planExecutionInfo.status == planExecutionInfo.SUCCESSFUL)?
                "executed successfully" : "aborted");
            RCLCPP_INFO(this->get_logger(), "Plan " + result_s + ": READY to execute new plan now\n");    
        }
    }else{
        //STILL running...
        
        //check if you've surpassed N times the estimated deadline (N ros2 parameter && >= 1.0)
        float cancelAfterDeadline = std::max(1.0f, (float) this->get_parameter(PARAM_CANCEL_AFTER_DEADLINE).as_double());
        if(planExecutionInfo.current_time >= cancelAfterDeadline * planExecutionInfo.target.deadline)
            cancelCurrentPlanExecution();
    }
}

/*
    Find earliest action, i.e. having earliest start timestamp within PlanSys2 executor client feedback
    vector of action exec. status
*/
ActionExecutionInfo extractEarliestAction(const vector<ActionExecutionInfo>& psys2_actions_status)
{
    if(psys2_actions_status.size() == 0)//empty array
        return ActionExecutionInfo();
    
    ActionExecutionInfo earliest_action;//take the first as earliest action
    earliest_action.start_stamp.sec = -1;

    for(int i = 0; i<psys2_actions_status.size(); i++)
    {
        if(psys2_actions_status[i].status != psys2_actions_status[i].NOT_EXECUTED)
            earliest_action =   (                                                                                   //update if
                                        (psys2_actions_status[i].start_stamp.sec < earliest_action.start_stamp.sec) //earliest sec ts
                                    ||                                                                              //OR
                                        (psys2_actions_status[i].start_stamp.sec == earliest_action.start_stamp.sec
                                            &&
                                        psys2_actions_status[i].start_stamp.nanosec < earliest_action.start_stamp.nanosec) // same sec ts AND earliest nanosec ts
                                    ||
                                        earliest_action.start_stamp.sec == -1                                       //earliest action still not init.
                                )?

                                psys2_actions_status[i] : earliest_action; //otherwise do not update
    }
    return earliest_action;
}

/* 
    Use PlanSys2 feedback received from the executor to build the BDIPlanExecutionInfo to be published to the respecive topic
    Call NECESSARY to update the properties regarding the status of the current monitored/managed plan exec.
*/
BDIPlanExecutionInfo PlanDirector::getPlanExecutionInfo(const ExecutorClient::ExecutePlan::Feedback& feedback)
{
    // retrieve plan body (action with duration and planned start step by step as computed by the pddl planner)
    vector<PlanItem> current_plan_body = current_plan_.toPsys2Plan().items;

    BDIPlanExecutionInfo planExecutionInfo = BDIPlanExecutionInfo();
    float status_time_s = -1.0;//current exec time relatively to plan start referred as the "zero" time point
    int executing = 0;
    
    if(first_ts_plan_sec_ < 0 && feedback.action_execution_status.size() > 0)//NOTE: update first ts for plan if it's not init yet and you've received the first significant feedback
    {
        //set just for earliest start timestamp captured in this plan exec (then always subtract from it)
        ActionExecutionInfo psys2_action_earliest = extractEarliestAction(feedback.action_execution_status);
        if(psys2_action_earliest.start_stamp.sec >= 0)
        {
            first_ts_plan_sec_ = psys2_action_earliest.start_stamp.sec;
            first_ts_plan_nanosec_ = psys2_action_earliest.start_stamp.nanosec;
        }
    }

    // find executing action status
    for (int i=0; i<current_plan_body.size(); i++) {
        string timex1000s = std::to_string(current_plan_body[i].time * 1000);
        timex1000s = timex1000s.substr(0, timex1000s.find_first_of("."));
        string action_full_name = current_plan_body[i].action + ":" + timex1000s;
        int aindex_psys2_feed = PDDLBDIConverter::getActionIndex(feedback.action_execution_status,  action_full_name);
        
        std::optional<ActionExecutionInfo> psys2_action_feed_opt = {};
        if(aindex_psys2_feed >= 0)
        {
            psys2_action_feed_opt = feedback.action_execution_status[aindex_psys2_feed];     
            executing += (psys2_action_feed_opt.value().status == psys2_action_feed_opt.value().EXECUTING)? 1 : 0;
        }   

        BDIActionExecutionInfo bdiActionExecutionInfo = PDDLBDIConverter::buildBDIActionExecutionInfo(psys2_action_feed_opt, current_plan_body, i,
                first_ts_plan_sec_, first_ts_plan_nanosec_);
        planExecutionInfo.actions_exec_info.push_back(bdiActionExecutionInfo);//add action execution info to plan execution info 

        // plan status time
        if(bdiActionExecutionInfo.status == bdiActionExecutionInfo.RUNNING)
            status_time_s = std::max(status_time_s, bdiActionExecutionInfo.actual_start + bdiActionExecutionInfo.exec_time);// actual start time for action + duration action up to now
    }

    //TODO sort before directly on Plansys2 feedback DO IT!!!!! SHOULD BE SOLVED
    sort(planExecutionInfo.actions_exec_info.begin(), planExecutionInfo.actions_exec_info.end(), 
        [](BDIActionExecutionInfo bdi_a1, BDIActionExecutionInfo bdi_a2){
            return bdi_a1.index < bdi_a2.index;
        }
    );
    
    planExecutionInfo.target = current_plan_.getPlanTarget().toDesire();
    planExecutionInfo.planned_deadline = current_plan_.getPlannedDeadline();
    planExecutionInfo.estimated_deadline = current_plan_.getUpdatedEstimatedDeadline();
    
    
    // current time s computed by difference from fist start ts of first action executed within the plan
    planExecutionInfo.current_time = (status_time_s >= 0.0f)? status_time_s : 0.0f;
    if(executing == 0 && last_ts_plan_exec_ > 0.0f)//last steps -> no action executing right now
        planExecutionInfo.current_time = last_ts_plan_exec_ + (PLAN_INTERVAL / pow(10,3)); //add plan interval in sec from last check
    last_ts_plan_exec_ = planExecutionInfo.current_time;
    planExecutionInfo.status = getPlanExecutionStatus();

    return planExecutionInfo;
}

/*
    Retrieve from PlanSys2 Executor status info about current plan execution: RUNNING, SUCCESSFUL, ABORT
*/
int16_t PlanDirector::getPlanExecutionStatus()
{
    int16_t result = BDIPlanExecutionInfo().RUNNING;
    if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) //plan stopped
    {      
        if(executor_client_->getResult().value().success)//successful  run
            result = BDIPlanExecutionInfo().SUCCESSFUL;
        
        else //plan aborted
            result = BDIPlanExecutionInfo().ABORT;
    }   
    return result;
}

/*
    The belief set has been updated
*/
void PlanDirector::updatedBeliefSet(const BeliefSet::SharedPtr msg)
{
    belief_set_ = BDIFilter::extractMGBeliefs(msg->value);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanDirector>();
  bool psys2_booted = node->wait_psys2_boot(std::chrono::seconds(8));//Wait max 8 seconds for plansys2 to boot
  
  if(psys2_booted)
  {
    node->init();
    rclcpp::spin(node);
  }
  else
  {
    std::cerr << "PlanSys2 failed to boot: node will not spin and process will terminate" << std::endl;
  }
  
  rclcpp::shutdown();

  return 0;
}
