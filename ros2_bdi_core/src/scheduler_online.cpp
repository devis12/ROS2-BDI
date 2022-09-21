// header file for SchedulerOnline node
#include "ros2_bdi_core/scheduler_online.hpp"
// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node (for plan exec srv & topic)
#include "ros2_bdi_core/params/plan_director_params.hpp"

/* Util classes */
#include "ros2_bdi_utils/BDIPDDLConverter.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"

#include "ros2_bdi_interfaces/msg/bdi_plan.hpp"

using std::string;
using std::vector;
using std::set;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::optional;

using plansys2::PlannerClient;
using plansys2::Goal;
using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::Condition;
using ros2_bdi_interfaces::msg::ConditionsConjunction;
using ros2_bdi_interfaces::msg::ConditionsDNF;
using ros2_bdi_interfaces::msg::BDIActionExecutionInfo;
using ros2_bdi_interfaces::msg::BDIPlanExecutionInfo;
using ros2_bdi_interfaces::msg::BDIActionExecutionInfoMin;
using ros2_bdi_interfaces::msg::BDIPlanExecutionInfoMin;
using ros2_bdi_interfaces::msg::BDIPlan;
using ros2_bdi_interfaces::srv::BDIPlanExecution;

using javaff_interfaces::msg::ExecutionStatus;
using javaff_interfaces::msg::CommittedStatus;
using javaff_interfaces::msg::SearchResult;
using javaff_interfaces::msg::PartialPlan;

using BDIManaged::ManagedDesire;
using BDIManaged::ManagedPlan;
using BDIManaged::ManagedCondition;
using BDIManaged::ManagedConditionsConjunction;
using BDIManaged::ManagedConditionsDNF;

using PlanLibrary::BDIPlanLibrary;


void SchedulerOnline::init()
{
    Scheduler::init();

    //init SchedulerOffline specific props
    fulfilling_desire_ = ManagedDesire{};
    waiting_plans_ = vector<ManagedPlan>();
    current_plan_ = ManagedPlan{};

    searching_ = false;

    executing_pplan_index_ = -1;//will be put to 0 as soon as next first computed and received pplan is launched for execution and then upd over time 
        
    // interval search ms
    this->declare_parameter(JAVAFF_SEARCH_INTERVAL_PARAM, JAVAFF_SEARCH_INTERVAL_PARAM_DEFAULT);

    // max null search interval ms
    this->declare_parameter(JAVAFF_SEARCH_MAX_EMPTY_SEARCH_INTERVALS_PARAM, JAVAFF_SEARCH_MAX_EMPTY_SEARCH_INTERVALS_PARAM_DEFAULT);

    //Topic where a desire to be augmented to currently active goal or added to the desire set is published
    boost_desire_subscriber_ = this->create_subscription<Desire>(
                BOOST_DESIRE_TOPIC, rclcpp::QoS(10).reliable(),
                bind(&SchedulerOnline::boostDesireTopicCallBack, this, _1));

    javaff_client_ = std::make_shared<JavaFFClient>(string("javaff_srvs_caller"));

    javaff_search_subscriber_ = this->create_subscription<SearchResult>(
        JAVAFF_SEARCH_TOPIC, rclcpp::QoS(10).reliable(),
            bind(&SchedulerOnline::updatedSearchResult, this, _1));

    executing_plan_committed_status_subscriber_ = this->create_subscription<CommittedStatus>(
        JAVAFF_COMMITTED_STATUS_TOPIC, rclcpp::QoS(10).reliable(),
            bind(&SchedulerOnline::updatedCommittedStatus, this, _1));

    
    //javaff_exec_status_publisher_ init
    javaff_exec_status_publisher_ = this->create_publisher<ExecutionStatus>(JAVAFF_EXEC_STATUS_TOPIC, 10);
    
    // open connection to plan library and init. tables, if not already present
    planlib_conn_ok_ = planlib_db_.initPlanLibrary();
}


/*
    Store plan in plan library && enqueue in waiting_plans
*/
void SchedulerOnline::storeEnqueuePlan(BDIManaged::ManagedPlan&mp)
{
    storePlan(mp);
    if(planlib_conn_ok_ && mp.getPlanLibID() >= 0)
    {
        //mp has been stored
        if(waiting_plans_.size() == 0 && current_plan_.getPlanLibID() >= 0)//mp is successor of current_plan_
            planlib_db_.markSuccessors(current_plan_, mp);
        
        else if(waitingPlansBack().value().getPlanLibID() >= 0)//mp is successor of current_plan_
            planlib_db_.markSuccessors(waitingPlansBack().value(), mp);
    }
    enqueuePlan(mp);

    //Log enqueue
    if(this->get_parameter(PARAM_DEBUG).as_bool())
    {
        string plan_queue_indexes = "";
        for(int i=0; i<waiting_plans_.size(); i++)
            plan_queue_indexes += std::to_string(waiting_plans_[i].getPlanQueueIndex()) + ", ";
        RCLCPP_INFO(this->get_logger(), "Enqueued plan with index " + std::to_string(mp.getPlanQueueIndex()) + "\n" + 
                            " Current waiting queue status: " + plan_queue_indexes + "\"");
    }
}

/*
    Store plan in plan library
*/
void SchedulerOnline::storePlan(BDIManaged::ManagedPlan&mp)
{
    if(planlib_conn_ok_)
    {
        int new_plan_id = planlib_db_.insertPlan(mp);
        if(new_plan_id >= 0)
            //plan has been stored successfully
            mp.setPlanLibID(new_plan_id);
        else
            planlib_conn_ok_ = false;
    }
}

/*
    Select plan execution based on precondition, deadline
*/
void SchedulerOnline::reschedule()
{   
    string reschedulePolicy = this->get_parameter(PARAM_RESCHEDULE_POLICY).as_string();
    bool noPlan = noPlanExecuting();
    if(reschedulePolicy == VAL_RESCHEDULE_POLICY_NO_IF_EXEC && !noPlan)//rescheduling not ammitted
        return;


    RCLCPP_INFO(this->get_logger(), "Online rescheduling");
    
    ManagedDesire selDesire = ManagedDesire{};

    vector<ManagedDesire> discarded_desires;

    mtx_iter_dset_.lock();//to sync between iteration in checkForSatisfiedDesires( ) && reschedule()

    for(ManagedDesire md : desire_set_)
    {   
        // FIRST BASIC RESCHEDULING selects highest priority desire which passes acceptance check, precondition check && has the highest priority atm
        TargetBeliefAcceptance validDesire = Scheduler::desireAcceptanceCheck(md);
        if(validDesire == ACCEPTED && 
            md.getPrecondition().isSatisfied(belief_set_) && 
            md.getPriority() > selDesire.getPriority())
            selDesire = md;
        
        else if(validDesire == UNKNOWN_INSTANCES)
        {
            planCompFailureHandler(md, false);//might still be valid in the future
            if(computed_plan_desire_map_[md.getName()] == this->get_parameter(PARAM_MAX_TRIES_COMP_PLAN).as_int())
                discarded_desires.push_back(md);
        }
        else if (validDesire == SYNTAX_ERROR || validDesire == UNKNOWN_PREDICATE)
            discarded_desires.push_back(md);//bound to be invalid desires
    }

    mtx_iter_dset_.unlock();//to sync between iteration in checkForSatisfiedDesires( ) && reschedule()
    
    //removed discarded desires
    for(ManagedDesire md : discarded_desires)
        delDesire(md);

    // I'm already searching for this
    if(searching_ && selDesire == fulfilling_desire_)
        return;
    
    RCLCPP_INFO(this->get_logger(), "Starting search for the fullfillment of Alex's desire to " + selDesire.getName());
    

    if(selDesire.getValue().size() > 0 && launchPlanSearch(selDesire))//a desire has effectively been selected && a search for it has been launched
    {    
        searching_ = true;
        search_baseline_ = emptySearchBaseline();
        RCLCPP_INFO(this->get_logger(), "Search started for the fullfillment of Alex's desire to " + selDesire.getName());
        fulfilling_desire_ = selDesire; 
    }
}

void SchedulerOnline::abortedPlanHandler(const bool handleDelete)
{
    string targetDesireName = fulfilling_desire_.getName();
    //current plan execution has been aborted
    int maxPlanExecAttempts = this->get_parameter(PARAM_MAX_TRIES_EXEC_PLAN).as_int();
    aborted_plan_desire_map_[targetDesireName]++;
    
    RCLCPP_INFO(this->get_logger(), "Plan execution for fulfilling desire \"" + targetDesireName + 
        "\" has been aborted for the %d time (max attempts: %d)", 
            aborted_plan_desire_map_[targetDesireName], maxPlanExecAttempts);
    
    if(handleDelete && aborted_plan_desire_map_[targetDesireName] >= maxPlanExecAttempts)
    {
        if(this->get_parameter(PARAM_DEBUG).as_bool())
            RCLCPP_INFO(this->get_logger(), "Desire \"" + targetDesireName + "\" will be removed because it doesn't seem feasible to fulfill it: too many plan abortions!");
        delDesire(fulfilling_desire_, true);
    }
}

/*
    Increment counter for failed computation of plans that aimed at fulfilling desire x
    DelDesire (and group) in case threshold is reached
*/
void SchedulerOnline::planCompFailureHandler(const BDIManaged::ManagedDesire& md, const bool handleDelete)
{
    int invCounter = ++computed_plan_desire_map_[md.getName()]; //increment invalid counter for this desire
    int maxTries = this->get_parameter(PARAM_MAX_TRIES_COMP_PLAN).as_int();
    string desireOperation = (invCounter < maxTries)? "desire will be rescheduled later" : "desire will be deleted from desire set";
    if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_INFO(this->get_logger(), "Desire \"" + md.getName() + "\" (or its preconditions): plan search failed; " +
        desireOperation + " (invalid counter = %d/%d).", invCounter, maxTries);

    if(handleDelete && invCounter >= maxTries)//desire needs to be discarded, because a plan has tried to be unsuccessfully computed for it too many times
        delDesire(md, true);
}

/*
    Init all info related to current desire in pursuit & plan to fulfill it
*/
void SchedulerOnline::resetSearchInfo()
{
    fulfilling_desire_ = ManagedDesire{};
    current_plan_ = ManagedPlan{};//no plan executing rn
    waiting_plans_ = vector<ManagedPlan>();
    searching_ = false;//saluti da T.V.
    search_baseline_ = emptySearchBaseline();
    executing_pplan_index_ = -1;//will be put to 0 as soon as next first computed and received pplan is launched for execution and then upd over time 

}

/*
    Init all info related to current desire in pursuit & plan to fulfill it, then launch reschedule
*/
void SchedulerOnline::forcedReschedule()
{
    resetSearchInfo();
    reschedule();
}

void SchedulerOnline::publishCurrentIntention()
{   
    BDIPlanExecutionInfoMin intentionMsg = BDIPlanExecutionInfoMin{};
    for(auto target_value : current_plan_.getFinalTarget().getValue())
        intentionMsg.target_value.push_back(target_value.toBelief());

    for(auto action : current_plan_.getActionsExecInfo())
    {
        BDIActionExecutionInfoMin ai_min = BDIActionExecutionInfoMin{};
        ai_min.name = action.name;
        ai_min.planned_start = action.planned_start;
        ai_min.progress = action.progress;
        ai_min.args = action.args;
        ai_min.committed = action.committed;
        ai_min.status = action.status;
        intentionMsg.actions_exec_info.push_back(ai_min);
    }

    for(int i = waiting_plans_.size() - 1; i >=0; i--)
    {
        auto plan = waiting_plans_[i];
        for(auto action : plan.getActionsExecInfo())
        {
            BDIActionExecutionInfoMin ai_min = BDIActionExecutionInfoMin{};
            ai_min.name = action.name;
            ai_min.planned_start = action.planned_start;
            ai_min.progress = action.progress;
            ai_min.args = action.args;
            ai_min.committed = action.committed;
            ai_min.status = action.status;
            intentionMsg.actions_exec_info.push_back(ai_min);
        }
    }

    intention_publisher_->publish(intentionMsg);
}

/*
    Received update on current plan execution
*/
void SchedulerOnline::updatePlanExecution(const BDIPlanExecutionInfo::SharedPtr msg)
{
    auto planExecInfo = (*msg);
    ManagedDesire planTargetDesire = ManagedDesire{planExecInfo.target};

    if(!noPlanExecuting() && planExecInfo.target.name == current_plan_.getPlanTarget().getName())//current plan selected in execution update
    {
        current_plan_exec_info_ = planExecInfo;
        current_plan_.setUpdatedInfo(planExecInfo);
        string targetDesireName = planTargetDesire.getName();
        
        if(planExecInfo.status != planExecInfo.RUNNING)
        {   
            // plan not running anymore
            ManagedDesire finalTarget = current_plan_.getFinalTarget();
            
            bool callUnexpectedState = false;

            if(planExecInfo.status == planExecInfo.SUCCESSFUL)//plan exec completed successful
            {
                current_plan_ = ManagedPlan{};//no plan executing rn

                //step over to the next pplan
                if(waiting_plans_.size() > 0)
                {
                    std::optional<ManagedPlan> nextPPlanToExec = dequeuePlan();
                    //launch plan execution
                    if(nextPPlanToExec.has_value() && nextPPlanToExec.value().getActionsExecInfo().size() > 0)
                    {
                        bool triggeredNewPlanExec;
                        triggeredNewPlanExec = tryTriggerPlanExecution(nextPPlanToExec.value());
                        if(triggeredNewPlanExec)
                            executing_pplan_index_ = nextPPlanToExec.value().getPlanQueueIndex();
                        callUnexpectedState = !triggeredNewPlanExec;//if failed, call unexpected state srv
                        
                        if(this->get_parameter(PARAM_DEBUG).as_bool())
                        {
                            string plan_queue_indexes = "";
                            for(int i=0; i<waiting_plans_.size(); i++)
                                plan_queue_indexes += std::to_string(waiting_plans_[i].getPlanQueueIndex()) + ", ";
                            if(triggeredNewPlanExec)
                                RCLCPP_INFO(this->get_logger(), "Started plan with index " + std::to_string(executing_pplan_index_) + "\n" + 
                                                    " Current waiting queue status: " + plan_queue_indexes + "\"");
                            else
                                RCLCPP_INFO(this->get_logger(), "Failed to start new plan with index " + std::to_string(nextPPlanToExec.value().getPlanQueueIndex()) + "\n" +
                                                    " Calling unexpectedState service\n" + 
                                                    " Current waiting queue status: " + plan_queue_indexes + "\"");
                        }
                    }
                }
                else if(!searching_)
                {           
                    //no plan left to execute && searching had already finished
                }
                else
                {
                    //TODO start a timer to wait for a result from the search, otherwise force reschedule
                    //This needs to be understood better
                }
            }

            //check if desire sat
            bool desireAchieved = isDesireSatisfied(finalTarget);
            if(desireAchieved)
            {
                publishTargetGoalInfo(DEL_GOAL_BELIEFS);
                mtx_iter_dset_.lock();
                delDesire(finalTarget, true);//desire achieved -> delete all desires within the same group
                mtx_iter_dset_.unlock();
                // reset all search related info
                resetSearchInfo();
                return;
            }

            if(!desireAchieved && (callUnexpectedState || planExecInfo.status == planExecInfo.ABORT))
            {
                callUnexpectedState = true;
                abortedPlanHandler();//increment counter for aborted plans that aimed at fulfilling desire x
            }

            if(callUnexpectedState)//handle the following with unexpected state srv
            {   
                publishTargetGoalInfo(DEL_GOAL_BELIEFS);
                //tmp cleaning //TODO need to be revised this after having fixed search full reset 
                current_plan_ = ManagedPlan{};//no plan executing rn
                waiting_plans_ = vector<ManagedPlan>();
                executing_pplan_index_ = -1;//will be put to 0 as soon as next first computed and received pplan is launched for execution and then upd over time 
                search_baseline_ = emptySearchBaseline();
                searching_ = javaff_client_->callUnexpectedStateSrv(problem_expert_->getProblem());
                if(!searching_)
                    forcedReschedule();//if service call failed, just reschedule from scratch solution!!!
            }
        }
    }
}

/* 
    compare passed search baseline with current one
        returns: 
            -1: if sb is less recent
            0:  if sb is at the same level (matching baselines)
            1:  if sb is more recent (more actions committed or greater executing plan index)
*/
int SchedulerOnline::compareBaseline(javaff_interfaces::msg::CommittedStatus sb)
{
    // std::cout << "compareBaseline: search_baseline_.pi = " << search_baseline_.executing_plan_index 
    //     << "\t sb.pi = " << sb.executing_plan_index << std::flush << std::endl;

    if(search_baseline_.executing_plan_index != sb.executing_plan_index)// compare executing plan index
        return (sb.executing_plan_index > search_baseline_.executing_plan_index)? 1 : -1;
    
    if(search_baseline_.committed_actions.size() != sb.committed_actions.size()) // diff plans' sizes
        return 1;//should be a different plan, more updated one: check this better in the future, but it should be the right assumption
    
    //count additional committed el in sb and search_baseline_
    int diff_committed1 = 0, diff_committed2 = 0;
    for(int i = 0; i < search_baseline_.committed_actions.size(); i++)//check action by action committed status
        if(search_baseline_.committed_actions[i].committed_action != sb.committed_actions[i].committed_action || search_baseline_.committed_actions[i].planned_start_time != sb.committed_actions[i].planned_start_time)
            return 1;//should be a different plan, more updated one: check this better in the future, but it should be the right assumption
        else if(search_baseline_.committed_actions[i].committed != sb.committed_actions[i].committed)
        {
            diff_committed1 += search_baseline_.committed_actions[i].committed? 1 : 0;
            diff_committed2 += sb.committed_actions[i].committed? 1 : 0;
        }
    
    // std::cout << "compareBaseline: search_baseline_.commit+ = " << diff_committed1 
    //     << "\t sb.commit+ = " << diff_committed2 << std::flush << std::endl;

    if(diff_committed1 > 0 || diff_committed2 > 0) // some diff detected
        if(diff_committed1 == diff_committed2) //same num of committed
            return -1; // consider search_baseline_ still more updated
        else
            return diff_committed2 > diff_committed1? 1 : -1;
    
    else // same committed situation    
        return 0; // matching baselines
}

/*
    Received update on current plan search
*/
void SchedulerOnline::updatedSearchResult(const SearchResult::SharedPtr msg)
{
    int more_upd_baseline = compareBaseline(msg->search_baseline); //-1 less upd, 0 matching_baseline, 1 more upd
    bool matching_baseline = more_upd_baseline == 0;// baselines are at the same level
    if(!matching_baseline && more_upd_baseline >= 0)// search baseline is NOT matching with previously received search result and is not less recent
        searching_ = true;// search has been started directly by javaff, because it detected the need to do it (e.g. non predicted changes which affects current plan execution)

    if(searching_)
    {
        if(msg->status != msg->SEARCHING)
            searching_ = false;//not searching anymore

        if(msg->status == msg->FAILED)
        {
            //search has failed!! increment counter, del desire if threshold reached and FORCE a reschedule
            planCompFailureHandler(fulfilling_desire_, true);
            forcedReschedule();
            return;
        }
        else
        {
            //search is progressing
            if(!noPlanExecuting() && !current_plan_.getFinalTarget().equalsOrSupersetIgnoreAdvancedInfo(fulfilling_desire_))//started a new search for a different desire -> should abort old executing plan
            {    
                if(abortCurrentPlanExecution())//if aborted is correctly performed, clean away the current waiting list as well, exploiting new msg to build the new one, for new instantiated search
                    resetSearchInfo();
            }

            if(matching_baseline) // search baseline is matching with previously received search result
            {
                // received incremental plans obtained through "search from scratch" or search with same search base line as last enqueued plan
                processIncrementalSearchResult(msg);
            }
            else if(more_upd_baseline == 1) // msg->search_baseline MORE_RECENT THAN search_baseline_
            {   
                // search baseline is NOT matching with previously received search results
                // received sub plan with a different search baseline wrt previous notification
                if(processSearchResultWithNewBaseline(msg))
                    search_baseline_ = msg->search_baseline;//update search baseline
                // javaff will stop curr search via exec status because it is too late compared to current exec status
            }
            
        }
    }   
}

/*
    Received update on current committed status for plan execution
*/
void SchedulerOnline::updatedCommittedStatus(const javaff_interfaces::msg::CommittedStatus::SharedPtr msg)
{
    if(current_plan_.getPlanQueueIndex() == msg->executing_plan_index)
    {
        for(auto acs : msg->committed_actions)
            current_plan_.setActionCommittedStatus(acs.committed_action, acs.planned_start_time, acs.committed);
    }
    publishCurrentIntention();
}

/*
    Regular process of updated search result with same search baseline as previous msgs or "original" status, when search was launched from scratch
*/
void SchedulerOnline::processIncrementalSearchResult(const javaff_interfaces::msg::SearchResult::SharedPtr msg)
{
    int i = 0;
    // store incremental partial plans 
    // as soon as you have the first, trigger plan execution
    int highestPPlanId = (waitingPlansBack().has_value())? waitingPlansBack().value().getPlanQueueIndex() : executing_pplan_index_;
    while(i<msg->plans.size() && msg->plans[i].plan.plan_index <= highestPPlanId){i++;}//go over all the search result till you reach the new ones
    //TODO check for plan exec and waiting queue inconsistencies, if detected, take action

    if(noPlanExecuting() && i<msg->plans.size() && msg->plans[i].plan.items.size() > 0)
    {  
        launchFirstPPlanExecution(msg->plans[i]); 
    }else if(msg->plans[i].plan.items.size() > 0){
        for(; i<msg->plans.size(); i++)//avoid considering first pplan which is demanded for execution in the if branch above
        {
            highestPPlanId = (waitingPlansBack().has_value())? waitingPlansBack().value().getPlanQueueIndex() : executing_pplan_index_;
            //to be enqueued must be higher than last waiting plan in queue or if queue is empty must be higher than the one currently in execution
            if(msg->plans[i].plan.plan_index > highestPPlanId)//should be enqueued
            {
                ManagedPlan computedMPP = ManagedPlan{msg->plans[i].plan.plan_index, fulfilling_desire_, ManagedDesire{msg->plans[i].target}, msg->plans[i].plan.items, ManagedConditionsDNF{msg->plans[i].target.precondition}, fulfilling_desire_.getContext()};
                storeEnqueuePlan(computedMPP);
            }
        }
        
    }
}

bool SchedulerOnline::launchFirstPPlanExecution(const PartialPlan& firstpplan)
{
    // insert subplan precondition as received by planner, desire precondition checked in scheduling
    ManagedPlan firstPPlanToExec = ManagedPlan{
        firstpplan.plan.plan_index, 
        fulfilling_desire_, 
        ManagedDesire{firstpplan.target}, firstpplan.plan.items, 
        ManagedConditionsDNF{firstpplan.target.precondition}, fulfilling_desire_.getContext()};
    //launch plan execution
    if(firstPPlanToExec.getActionsExecInfo().size() > 0)
    {   
        storePlan(firstPPlanToExec);
        bool triggered = tryTriggerPlanExecution(firstPPlanToExec);
        if(this->get_parameter(PARAM_DEBUG).as_bool())
        {
            if(triggered) RCLCPP_INFO(this->get_logger(), "Triggered new plan execution success");
            else RCLCPP_INFO(this->get_logger(), "Triggered new plan execution failed");
        }

        if(!triggered)// just reschedule from scratch
        {
            abortedPlanHandler();
            forcedReschedule();
            return false;
        }
        else
            executing_pplan_index_ = firstPPlanToExec.getPlanQueueIndex();
    }
    else
        return false;

    return true;
}

/*
    Process updated search result presenting a new search baseline compared to previous msgs of the same type
*/
bool SchedulerOnline::processSearchResultWithNewBaseline(const javaff_interfaces::msg::SearchResult::SharedPtr msg)
{
    // processSearchResultWithNewBaseline
    //      check if still feasible wrt. search_baseline and request early abort.
    //      If early abort success -> replace waiting_plan with new search result

    Plan psys2_current_plan = current_plan_.toPsys2Plan();

    if(psys2_current_plan.items.size() > 0 && psys2_current_plan.items.size() != msg->search_baseline.committed_actions.size())
    {
        return false; // plans do not match: cannot be handled
    }
    
    // std::cout << "SchedulerOnline::processSearchResultWithNewBaseline\npindex: " << std::to_string(msg->search_baseline.executing_plan_index) << std::flush << std::endl;
    // for(int i=0; i<msg->search_baseline.committed_actions.size(); i++)
    // {
    //     std::cout <<  "- " << msg->search_baseline.committed_actions[i].committed_action << ":" << 
    //         std::to_string(static_cast<int>(msg->search_baseline.committed_actions[i].planned_start_time*1000)) <<
    //         "\t" << std::to_string(msg->search_baseline.committed_actions[i].committed) << std::flush << std::endl;
    // }
    
    int committed_counter = 0;
    for(int i=0; i<psys2_current_plan.items.size(); i++)
    {
        psys2_current_plan.items[i].committed = msg->search_baseline.committed_actions[i].committed;
        if(psys2_current_plan.items[i].committed)
            committed_counter++;
    }

    bool successful_update = false;// it was possible to process successfully the search result with new baseline (i.e. not too late)

    bool early_abort_request_success = false;
    if(committed_counter < psys2_current_plan.items.size())
    {
        //in this case it make sense to do an early abort request
        BDIPlan early_abort_bdiplan = BDIPlan{};
        early_abort_bdiplan.target = current_plan_.getPlanTarget().toDesire();
        early_abort_bdiplan.precondition = current_plan_.getPrecondition().toConditionsDNF();
        early_abort_bdiplan.psys2_plan = psys2_current_plan;
        early_abort_bdiplan.context = current_plan_.getContext().toConditionsDNF();
        early_abort_request_success = plan_exec_srv_client_->earlyArrestRequest(early_abort_bdiplan);
        if(this->get_parameter(PARAM_DEBUG).as_bool())
            if(early_abort_request_success) RCLCPP_INFO(this->get_logger(), "Early arrest request: ACCEPTED");
            else RCLCPP_INFO(this->get_logger(), "Early arrest request: TOO LATE!");
    }

    if(committed_counter == psys2_current_plan.items.size() || early_abort_request_success)
    {
        successful_update = true;
        int i = msg->base_plan_index;

        if(noPlanExecuting() && msg->plans[i].plan.items.size() > 0)
        {  
            successful_update = launchFirstPPlanExecution(msg->plans[i]); 
            i++;
        }

        if(successful_update)
        {
            // substitute waiting queue
            waiting_plans_.clear();
            while(i < msg->plans.size())
            {
                ManagedPlan computedMPP = ManagedPlan{msg->plans[i].plan.plan_index, 
                    fulfilling_desire_, 
                    ManagedDesire{msg->plans[i].target}, msg->plans[i].plan.items, 
                    ManagedConditionsDNF{msg->plans[i].target.precondition}, 
                    fulfilling_desire_.getContext()};
                storeEnqueuePlan(computedMPP);
                i++;
            }
        }
        
    }

    return successful_update;
}


/*
    Launch a new plan search
*/
bool SchedulerOnline::launchPlanSearch(const BDIManaged::ManagedDesire& selDesire)
{
    //set desire as goal of the pddl_problem
    if(!problem_expert_->setGoal(Goal{BDIPDDLConverter::desireToGoal(selDesire.toDesire())})){
        //psys2_comm_errors_++;//plansys2 comm. errors
        return false;
    }

    string pddl_problem = problem_expert_->getProblem();//get problem string
    int intervalSearchMS = this->get_parameter(JAVAFF_SEARCH_INTERVAL_PARAM).as_int();
    int maxEmptySearchIntervals = this->get_parameter(JAVAFF_SEARCH_MAX_EMPTY_SEARCH_INTERVALS_PARAM).as_int();
    intervalSearchMS = intervalSearchMS >= 100? intervalSearchMS : 100;
    maxEmptySearchIntervals = maxEmptySearchIntervals > 0? maxEmptySearchIntervals : 16;
    return javaff_client_->launchPlanSearch(selDesire.toDesire(), pddl_problem, intervalSearchMS, maxEmptySearchIntervals);
}

/*
    wrt the current plan execution...
    return sum of progress status of all actions within a plan divided by the number of actions
*/
float SchedulerOnline::computePlanProgressStatus()
{   
    // not exeuting any plan or last update not related to currently triggered plan
    if(noPlanExecuting() || current_plan_exec_info_.target.name != current_plan_.getPlanTarget().getName() || current_plan_exec_info_.actions_exec_info.size() == 0)
        return 0.0f;

    float progress_sum = 0.0f;
    for(auto currentExecutingAction : current_plan_exec_info_.actions_exec_info)
        progress_sum += currentExecutingAction.progress;

    return progress_sum/(current_plan_exec_info_.actions_exec_info.size());
}

/*  Use the updated belief set for deciding if some desires are pointless to pursue given the current 
    beliefs which shows they're already fulfilled
*/
void SchedulerOnline::checkForSatisfiedDesires()
{
    mtx_iter_dset_.lock();//to sync between iteration in checkForSatisfiedDesires( ) && reschedule()

    vector<ManagedDesire> satisfiedDesires;
    for(ManagedDesire md : desire_set_)
    {   
        if(isDesireSatisfied(md))//desire already achieved, remove it
        {
            if(!noPlanExecuting() && current_plan_.getFinalTarget() == md && 
                current_plan_exec_info_.status == current_plan_exec_info_.RUNNING)  
            {
                float plan_progress_status = computePlanProgressStatus();
                
                if(plan_progress_status < COMPLETED_THRESHOLD)
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Current plan execution fulfilling desire \"" + md.getName() + 
                            "\" will be aborted since desire is already fulfilled and plan exec. is still far from being completed " +
                            "(progress status = %f)", plan_progress_status);

                    if(abortCurrentPlanExecution())
                        resetSearchInfo();
                }
            }
            else
            {
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Desire \"" + md.getName() + "\" will be removed from the desire set since its "+
                        "target appears to be already fulfilled given the current belief set");
            
                satisfiedDesires.push_back(md);//delete desire just if not executing one, otherwise will be deleted when aborted feedback comes and desire is satisfied
            }

        }
    }

    for(ManagedDesire md : satisfiedDesires)//delete all satisfied desires from desire set
        delDesire(md, true);//you're deleting satisfied desires -> delete all the ones in the same group

    mtx_iter_dset_.unlock();//to sync between iteration in checkForSatisfiedDesires( ) && reschedule()
}

/*
    Process desire boost request for active goal augmentation
*/
void SchedulerOnline::boostDesireTopicCallBack(const Desire::SharedPtr msg)
{
    ManagedDesire mdBoost = ManagedDesire{(*msg)};
    if(fulfilling_desire_.getValue().size() == 0)
    {
        //no currently active desire
        bool result = addDesire(mdBoost);
    }
    else if (mdBoost.getName() == fulfilling_desire_.getName())
    {
        bool boosted = false;
        if(mdBoost.getPrecondition().isSatisfied(belief_set_) && mdBoost.getContext().isSatisfied(belief_set_))
        {
            // perform online boost
            ManagedDesire original_desire = fulfilling_desire_.clone();
            boosted = fulfilling_desire_.boostDesire(mdBoost);
            if(boosted)
            {
                if(problem_expert_->setGoal(Goal{BDIPDDLConverter::desireToGoal(fulfilling_desire_.toDesire())}))
                {
                    replaceDesire(original_desire, fulfilling_desire_);
                    
                    //inform planner, by pub a new execution status
                    ExecutionStatus boosted_goal_msg = ExecutionStatus{};
                    boosted_goal_msg.executing_plan_index = executing_pplan_index_;
                    boosted_goal_msg.pddl_problem = problem_expert_->getProblem();
                    boosted_goal_msg.sim_to_goal = boosted_goal_msg.SIM_TO_GOAL_FORCE_REPLAN;
                    boosted_goal_msg.notification_reason = boosted_goal_msg.GOAL_BOOST;
                    javaff_exec_status_publisher_->publish(boosted_goal_msg);
                    // Sim to goal should fail, producing an adjust course of actions to tackle and fulfill boosted goal
                }
                else
                {
                    //go back to original desire
                    fulfilling_desire_ = original_desire;
                } 
            }
        }
        
        if(!boosted)
        {
            int16_t instance_counter = 2;
            while(computed_plan_desire_map_.count(mdBoost.getName()+std::to_string(instance_counter)) == 0) instance_counter++;
            string new_name = mdBoost.getName()+std::to_string(instance_counter);
            mdBoost.setName(new_name); // set new name, to distinguish it from the original, by putting the first av. number at the end
            bool result = addDesire(mdBoost);//then add it as a separate desire
        }
    }
    else if(computed_plan_desire_map_.count(mdBoost.getName()) == 0)
    {
        // it does not match the currently active desire nor any other desire in the desire set, simply add it to the desire set
        bool result = addDesire(mdBoost);
    }
    else
    {
        bool result1 = false, result2 = false;
        // it does match a desire in the desire set which is currently not active, perform offline boost
        for(ManagedDesire md : desire_set_)
            if(md.getName() == mdBoost.getName())
            {
                ManagedDesire original_desire = md.clone();
                result1 = md.boostDesire(mdBoost);
                if(!result1)
                    result2 = addDesire(md);
                else
                {
                    replaceDesire(original_desire, md);
                }
                break;
            }
        
    }
}


/*
    Specific behaviour of scheduler after desire successful addition, based on its selected mode    
*/
void SchedulerOnline::postAddDesireSuccess(const BDIManaged::ManagedDesire& md)
{
    // Offline mode behaviour
    checkForSatisfiedDesires();// check for desire to be already fulfilled

    if(state_ == SCHEDULING && desire_set_.size() > 0 && noPlanExecuting())// still there to be satisfied && no plan selected, rescheduled immediately
    {   
        reschedule();
    }
}

/*
    Specific behaviour of scheduler after desire successful deletion, based on its selected mode    
*/
void SchedulerOnline::postDelDesireSuccess(const BDIManaged::ManagedDesire& md)
{
    //Offline mode behaviour
    if(md == current_plan_.getFinalTarget())//deleted desire of current executing plan)
    {
        //ABORT CURRENT AND WAITING PLANS
        if(abortCurrentPlanExecution())
            resetSearchInfo();
    }
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); 
  auto node = std::make_shared<SchedulerOnline>();
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
