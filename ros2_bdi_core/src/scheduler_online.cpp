// header file for SchedulerOnline node
#include "ros2_bdi_core/scheduler_online.hpp"
// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node (for plan exec srv & topic)
#include "ros2_bdi_core/params/plan_director_params.hpp"

/* Util classes */
#include "ros2_bdi_utils/BDIPDDLConverter.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"

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
using ros2_bdi_interfaces::srv::BDIPlanExecution;

using javaff_interfaces::msg::SearchResult;

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

    javaff_client_ = std::make_shared<JavaFFClient>(string("javaff_srvs_caller"));

    javaff_search_subscriber_ = this->create_subscription<SearchResult>(
        JAVAFF_SEARCH_TOPIC, 10,
            bind(&SchedulerOnline::updatedIncrementalPlan, this, _1));
    
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
    
    BDIManaged::ManagedDesire selDesire;

    mtx_iter_dset_.lock();//to sync between iteration in checkForSatisfiedDesires( ) && reschedule()

    for(ManagedDesire md : desire_set_)
    {   
        // FIRST BASIC RESCHEDULING selects highest priority desire which passes acceptance check, precondition check && has the highest priority atm
        if(Scheduler::desireAcceptanceCheck(md) == ACCEPTED && 
            md.getPrecondition().isSatisfied(belief_set_) && 
            md.getPriority() > selDesire.getPriority())
            selDesire = md;
    }

    mtx_iter_dset_.unlock();//to sync between iteration in checkForSatisfiedDesires( ) && reschedule()
    
    // I'm already searching for this
    if(searching_ && selDesire == fulfilling_desire_)
        return;
    
    RCLCPP_INFO(this->get_logger(), "Starting search for the fullfillment of Alex's desire to " + selDesire.getName());
    

    if(selDesire.getValue().size() > 0 && launchPlanSearch(selDesire))//a desire has effectively been selected && a search for it has been launched
    {    
        searching_ = true;
        RCLCPP_INFO(this->get_logger(), "Search started for the fullfillment of Alex's desire to " + selDesire.getName());
        fulfilling_desire_ = selDesire; 
    }
}

/*
    Init all info related to current desire in pursuit & plan to fulfill it, then launch reschedule
*/
void SchedulerOnline::forcedReschedule()
{
    current_plan_ = ManagedPlan{};//no plan executing rn
    waiting_plans_ = vector<ManagedPlan>();
    searching_ = false;//saluti da T.V.
    executing_pplan_index_ = -1;//will be put to 0 as soon as next first computed and received pplan is launched for execution and then upd over time 

    reschedule();
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
        if(planExecInfo.status != planExecInfo.RUNNING)//plan not running anymore
        {
            bool callUnexpectedState = false;

            if(planExecInfo.status == planExecInfo.SUCCESSFUL)//plan exec completed successful
            {
                //step over to the next pplan
                if(waiting_plans_.size() > 0)
                {
                    std::optional<ManagedPlan> nextPPlanToExec = dequeuePlan();
                    //launch plan execution
                    if(nextPPlanToExec.has_value() && nextPPlanToExec.value().getActionsExecInfo().size() > 0)
                    {
                        bool triggeredNewPlanExec;
                        current_plan_ = ManagedPlan{};//no plan executing rn
                        triggeredNewPlanExec = tryTriggerPlanExecution(nextPPlanToExec.value());
                        if(triggeredNewPlanExec)
                            executing_pplan_index_ = nextPPlanToExec.value().getPlanQueueIndex();
                        callUnexpectedState = !triggeredNewPlanExec;//if failed, call unexpected state srv
                        if(this->get_parameter(PARAM_DEBUG).as_bool())
                        {
                            if(triggeredNewPlanExec) RCLCPP_INFO(this->get_logger(), "Triggered new plan execution success");
                            else RCLCPP_INFO(this->get_logger(), "Triggered new plan execution failed");
                        }
                    }
                }
                else
                {   
                    current_plan_ = ManagedPlan{};//no plan executing rn
                    executing_pplan_index_ = -1;//will put to 0 as soon as next first computed and received pplan is launched for execution and then upd over time 
        
                    //no plan left to execute
                    ManagedDesire finalTarget = current_plan_.getFinalTarget();
                    mtx_iter_dset_.lock();
                    bool desireAchieved = isDesireSatisfied(finalTarget);
                    if(desireAchieved)
                        delDesire(finalTarget, true);//desire achieved -> delete all desires within the same group
                    mtx_iter_dset_.unlock();
                }
            }

            if(planExecInfo.status == planExecInfo.ABORT)
            {
                callUnexpectedState = true;

                string targetDesireName = fulfilling_desire_.getName();
                //current plan execution has been aborted
                int maxPlanExecAttempts = this->get_parameter(PARAM_MAX_TRIES_EXEC_PLAN).as_int();
                aborted_plan_desire_map_[targetDesireName]++;
                
                RCLCPP_INFO(this->get_logger(), "Plan execution for fulfilling desire \"" + targetDesireName + 
                    "\" has been aborted for the %d time (max attempts: %d)", 
                        aborted_plan_desire_map_[targetDesireName], maxPlanExecAttempts);
                
                if(aborted_plan_desire_map_[targetDesireName] >= maxPlanExecAttempts)
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Desire \"" + targetDesireName + "\" will be removed because it doesn't seem feasible to fulfill it: too many plan abortions!");
                    delDesire(fulfilling_desire_, true);
                }
            }

            if(callUnexpectedState)//handle the following with unexpected state srv
            {        
                //tmp cleaning //TODO need to be revised this after having fixed search full reset 
                current_plan_ = ManagedPlan{};//no plan executing rn
                waiting_plans_ = vector<ManagedPlan>();
                executing_pplan_index_ = -1;//will be put to 0 as soon as next first computed and received pplan is launched for execution and then upd over time 
                searching_ = javaff_client_->callUnexpectedStateSrv(problem_expert_->getProblem());
                if(!searching_)
                    forcedReschedule();//if service call failed, just reschedule from scratch solution!!!
            }
        }
    }
}

/*
    Received update on current plan search
*/
void SchedulerOnline::updatedIncrementalPlan(const SearchResult::SharedPtr msg)
{
    if(searching_)
    {
        if(msg->status != msg->SEARCHING)
            searching_ = false;//not searching anymore

        if(msg->status == msg->FAILED)
        {
            //search has failed!!
            int invCounter = ++computed_plan_desire_map_[fulfilling_desire_.getName()]; //increment invalid counter for this desire
            int maxTries = this->get_parameter(PARAM_MAX_TRIES_COMP_PLAN).as_int();
            string desireOperation = (invCounter < maxTries)? "desire will be rescheduled later" : "desire will be deleted from desire set";
            if(this->get_parameter(PARAM_DEBUG).as_bool())
                RCLCPP_INFO(this->get_logger(), "Desire \"" + fulfilling_desire_.getName() + "\" (or its preconditions): plan search failed; " +
                desireOperation + " (invalid counter = %d/%d).", invCounter, maxTries);
        
            if(invCounter >= maxTries)//desire needs to be discarded, because a plan has tried to be unsuccessfully computed for it too many times
                delDesire(fulfilling_desire_, true);
            
            forcedReschedule();
            return;
        }
        else
        {
            //search is progressing
            if(!noPlanExecuting() && current_plan_.getFinalTarget() != fulfilling_desire_)//started a new search for a different desire -> should abort old executing plan
            {    
                if(abortCurrentPlanExecution())//if aborted is correctly performed, clean away the current waiting list as well, exploiting new msg to build the new one, for new instantiated search
                    waiting_plans_ = vector<ManagedPlan>();
            }
            
            int i = 0;
            // store incremental partial plans 
            // as soon as you have the first, trigger plan execution
            
            if(noPlanExecuting() && msg->plans.size() > 0)
            {  
                // make union of high level desire precondition and subplan precondition
                ConditionsDNF allPreconditions = fulfilling_desire_.getPrecondition().toConditionsDNF();
                allPreconditions.clauses.insert(allPreconditions.clauses.end(), msg->plans[i].target.precondition.clauses.begin(),msg->plans[i].target.precondition.clauses.end());
                ManagedPlan firstPPlanToExec = ManagedPlan{fulfilling_desire_, ManagedDesire{msg->plans[i].target}, msg->plans[i].plan.items, ManagedConditionsDNF{allPreconditions}, fulfilling_desire_.getContext()};
                firstPPlanToExec.setPlanQueueIndex(msg->plans[i].index);
                storePlan(firstPPlanToExec);

                //launch plan execution
                if(firstPPlanToExec.getActionsExecInfo().size() > 0)
                {   
                    bool triggered = tryTriggerPlanExecution(firstPPlanToExec);
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                    {
                        if(triggered) RCLCPP_INFO(this->get_logger(), "Triggered new plan execution success");
                        else RCLCPP_INFO(this->get_logger(), "Triggered new plan execution failed");
                    }

                    if(!triggered)// just reschedule from scratch
                    {
                       forcedReschedule();
                       return;
                    }
                    else
                        executing_pplan_index_ = firstPPlanToExec.getPlanQueueIndex();
                }
                i++;
            
            }else if(msg->plans[i].plan.items.size() > 0){
                for(i = 1; i<msg->plans.size(); i++)//avoid considering first pplan which is demanded for execution in the if branch above
                {
                    int queueHighestPPlanId = (waitingPlansBack().has_value())? waitingPlansBack().value().getPlanQueueIndex() : 0;
                    //to be enqueued must be higher than last waiting plan in queue or if queue is empty must be higher than the one currently in execution
                    if(waiting_plans_.empty() && msg->plans[i].index > executing_pplan_index_ || msg->plans[i].index > queueHighestPPlanId)//should be enqueued
                    {
                        ManagedPlan computedMPP = ManagedPlan{fulfilling_desire_, ManagedDesire{msg->plans[i].target}, msg->plans[i].plan.items, ManagedConditionsDNF{msg->plans[i].target.precondition}, fulfilling_desire_.getContext()};
                        computedMPP.setPlanQueueIndex(msg->plans[i].index);//set plan queue index retrieved by planner
                        storeEnqueuePlan(computedMPP);
                    }
                }
                
            }
        }
    }   
}

/*
    Launch a new plan search
*/
bool SchedulerOnline::launchPlanSearch(const BDIManaged::ManagedDesire selDesire)
{
    //set desire as goal of the pddl_problem
    if(!problem_expert_->setGoal(Goal{BDIPDDLConverter::desireToGoal(selDesire.toDesire())})){
        psys2_comm_errors_++;//plansys2 comm. errors
        return false;
    }

    string pddl_problem = problem_expert_->getProblem();//get problem string
    int intervalSearchMS = this->get_parameter(JAVAFF_SEARCH_INTERVAL_PARAM).as_int();
    intervalSearchMS = intervalSearchMS >= 100? intervalSearchMS : 100;
    return javaff_client_->launchPlanSearch(selDesire.toDesire(), pddl_problem, intervalSearchMS);
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
                    {
                        searching_ = false;
                        waiting_plans_ = vector<ManagedPlan>();
                    }
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
        {
            searching_ = false;
            waiting_plans_ = vector<ManagedPlan>();
        }
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
