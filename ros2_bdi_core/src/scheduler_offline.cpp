// header file for SchedulerOffline node
#include "ros2_bdi_core/scheduler_offline.hpp"
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

using BDIManaged::ManagedDesire;
using BDIManaged::ManagedPlan;


void SchedulerOffline::init()
{
    Scheduler::init();

    //init SchedulerOffline specific props
    current_plan_ = ManagedPlan{};
}

/*
    Compute plan from managed desire, setting its belief array representing the desirable state to reach
    as the goal of the PDDL problem 
*/
optional<Plan> SchedulerOffline::computePlan(const ManagedDesire& md)
{   
    //set desire as goal of the pddl_problem
    if(!problem_expert_->setGoal(Goal{BDIPDDLConverter::desireToGoal(md.toDesire())})){
        //psys2_comm_errors_++;//plansys2 comm. errors
        return std::nullopt;
    }

    string pddl_domain = domain_expert_->getDomain();//get domain string
    string pddl_problem = problem_expert_->getProblem();//get problem string
    return planner_client_->getPlan(pddl_domain, pddl_problem);//compute plan (n.b. goal unfeasible -> plan not computed)
}

/*
    Select plan execution based on precondition, deadline
*/
void SchedulerOffline::reschedule()
{   
    string reschedulePolicy = this->get_parameter(PARAM_RESCHEDULE_POLICY).as_string();
    bool noPlan = noPlanExecuting();
    if(reschedulePolicy == VAL_RESCHEDULE_POLICY_NO_IF_EXEC && !noPlan)//rescheduling not ammitted
        return;
    
    //rescheduling possible, but plan currently in exec (substitute just for plan with higher priority)
    bool planinExec = reschedulePolicy != VAL_RESCHEDULE_POLICY_NO_IF_EXEC && !noPlan;

    // priority of selected plan
    float highestPriority = -1.0f;
    // deadline of selected plan
    float selectedDeadline = -1.0f;//  init to negative value
    
    ManagedPlan selectedPlan;
    
    vector<ManagedDesire> discarded_desires;
    
    mtx_iter_dset_.lock();//to sync between iteration in checkForSatisfiedDesires( ) && reschedule()

    set<ManagedDesire> skip_desires;

    for(ManagedDesire md : desire_set_)
    {
        if(skip_desires.count(md) == 1)
            continue;

        //desire currently fulfilling
        if(current_plan_.getFinalTarget() == md)
            continue;
        
        //plan in exec has higher priority than this one, skip this desire
        if(planinExec && current_plan_.getFinalTarget().getPriority() > md.getPriority())
            continue;
        
        bool computedPlan = false;//flag to mark plan for desire as computable
        bool invalidDesire = false;//flag to mark invalid desire
        
        // select just desires with satisyfing precondition and 
        // with higher or equal priority with respect to the one currently selected
        bool explicitPreconditionSatisfied = md.getPrecondition().isSatisfied(belief_set_);
        if(explicitPreconditionSatisfied && md.getPriority() >= highestPriority){
            optional<Plan> opt_p = computePlan(md);
            if(opt_p.has_value())
            {
                computedPlan = true;

                ManagedPlan mp = ManagedPlan{0, md, opt_p.value().items, md.getPrecondition(), md.getContext()};
                // does computed deadline for this plan respect desire deadline?
                if(mp.getPlannedDeadline() <= md.getDeadline()) 
                {
                    // pick it as selected plan iff: no plan selected yet || desire has higher priority than the one selected
                    // or equal priority, but smaller deadline
                    if(selectedDeadline < 0 || md.getPriority() > highestPriority || mp.getPlannedDeadline() < selectedDeadline)
                    {    
                        selectedDeadline = mp.getPlannedDeadline();
                        highestPriority = md.getPriority();
                        selectedPlan = mp;
                    
                    }else if(md.getPriority() < highestPriority && this->get_parameter(PARAM_DEBUG).as_bool()){
                        RCLCPP_INFO(this->get_logger(), "There is a plan to fulfill desire \"" + md.getName() + "\", but "+ 
                            "it it's not the desire (among which a plan can be selected) with highest priority right now");

                    }else if(mp.getPlannedDeadline() >= selectedDeadline && this->get_parameter(PARAM_DEBUG).as_bool()){
                        RCLCPP_INFO(this->get_logger(), "There is a plan to fulfill desire \"" + md.getName() + "\", but "+
                            "it it's not the desire (among which a plan can be selected) with highest priority and earliest deadline right now");
                    }

                }else if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "There is a plan to fulfill desire \"" + md.getName() + "\", but it does not respect the deadline constraint");

            }   
            else if(Scheduler::desireAcceptanceCheck(md) != ACCEPTED) //check if the problem is the goal not being valid          
            {
                invalidDesire = true;
            }   
            else 
            {
                if(!this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Desire \"" + md.getName() + "\" presents a valid goal, but planner cannot compute any plan for it at the moment");
            }
        }
        else if(!explicitPreconditionSatisfied && this->get_parameter(PARAM_AUTOSUBMIT_PREC).as_bool())
        {
            int pushed = 0;
            if(Scheduler::desireAcceptanceCheck(md) == ACCEPTED)
            {
                // explicit preconditions are not satisfied... see if it's feasible to compute a plan to reach them
                // (just if not already done... that's why you look into the invalid map)
                // if it is the case submit desire to itself with higher priority than the one just considered 
                string fulfillPreconditionDesireName = md.getName() + "_fulfill_precondition";

                //put slightly lower priority because these would be desires for satisfy the pre precondition
                vector<ManagedDesire> fulfillPreconditionDesires = BDIFilter::conditionsToMGDesire(md.getPrecondition(), 
                    fulfillPreconditionDesireName, 
                    std::min(md.getPriority()-0.01f, 1.0f), md.getDeadline());
                
                for(ManagedDesire fulfillPreconditionD : fulfillPreconditionDesires)
                {   
                    auto precAcceptanceCheck = Scheduler::desireAcceptanceCheck(fulfillPreconditionD);
                    if(desire_set_.count(fulfillPreconditionD) == 0 &&  precAcceptanceCheck == ACCEPTED)//fulfill precondition not inserted yet
                    {   
                        if(this->get_parameter(PARAM_DEBUG).as_bool())
                            RCLCPP_INFO(this->get_logger(), "Precondition are not satisfied for desire \"" + md.getName() + "\" but could be satisfied: " +  
                                +  " auto-submission desire \"" + fulfillPreconditionD.getName() + "\"");
                        fulfillPreconditionD.setParent(md);//set md as its parent desire
                        if(addDesire(fulfillPreconditionD, md, "_preconditions"))
                            pushed++;
                    }
                }
            }

            if(pushed == 0)
                invalidDesire = true;
            
        }
        
        
        if(invalidDesire || (!computedPlan && explicitPreconditionSatisfied))
        {
            int invCounter = ++computed_plan_desire_map_[md.getName()]; //increment invalid counter for this desire
            int maxTries = this->get_parameter(PARAM_MAX_TRIES_COMP_PLAN).as_int();

            TargetBeliefAcceptance desAcceptance = desireAcceptanceCheck(md);
            string desireProblem = (desAcceptance != ACCEPTED)? "invalid goal" : "plan not computable";
            string desireOperation = (invCounter < maxTries && (desAcceptance == ACCEPTED || desAcceptance == UNKNOWN_INSTANCES))? 
                "desire will be rescheduled later" : "desire will be deleted from desire set";
            if(this->get_parameter(PARAM_DEBUG).as_bool())
                RCLCPP_INFO(this->get_logger(), "Desire \"" + md.getName() + "\" (or its preconditions):" +  desireProblem + " ; " +
                    desireOperation + " (invalid counter = %d/%d). " + std::to_string(desAcceptance), invCounter, maxTries);
            
            if(invCounter >= maxTries || (desAcceptance != ACCEPTED &&  desAcceptance != UNKNOWN_INSTANCES))//desire now has to be discarded
            {    
                discarded_desires.push_back(md);// plan to delete desire from desire_set (not doing here because we're cycling on desire)
                
                // check if this is trying to satisfy precondition and/or context condition of another desire
                // and there are no other desires within the same group -> delete that desire too
                int groupCounter = 0;
                for(ManagedDesire mdCheck : desire_set_)
                    if(mdCheck.getDesireGroup() == md.getDesireGroup())
                        groupCounter ++;
                if(md.hasParent() && groupCounter == 0)//invalid desire has remained the last one in the group
                {
                    discarded_desires.push_back(md.getParent());//parent cannot be satisfied either
                    skip_desires.insert(md.getParent());//avoid to evaluate it later
                }
            }
        }
        
    }

    mtx_iter_dset_.unlock();//to sync between iteration in checkForSatisfiedDesires( ) && reschedule()

    //removed discarded desires
    for(ManagedDesire md : discarded_desires)
        delDesire(md);

    if(selectedPlan.getActionsExecInfo().size() > 0)
    {
        bool triggered = tryTriggerPlanExecution(selectedPlan);
        if(this->get_parameter(PARAM_DEBUG).as_bool())
        {
            if(triggered) RCLCPP_INFO(this->get_logger(), "Triggered new plan execution success");
            else RCLCPP_INFO(this->get_logger(), "Triggered new plan execution failed");
        }
    }
}

/*
    Received update on current plan execution
*/
void SchedulerOffline::updatePlanExecution(const BDIPlanExecutionInfo::SharedPtr msg)
{
    auto planExecInfo = (*msg);
    ManagedDesire targetDesire = ManagedDesire{planExecInfo.target};

    if(!noPlanExecuting() && planExecInfo.target.name == current_plan_.getFinalTarget().getName())//current plan selected in execution update
    {
        current_plan_exec_info_ = planExecInfo;
        current_plan_.setUpdatedInfo(planExecInfo);

        string targetDesireName = targetDesire.getName();
        if(planExecInfo.status != planExecInfo.RUNNING)//plan not running anymore
        {
            mtx_iter_dset_.lock();
            bool desireAchieved = isDesireSatisfied(targetDesire);
            if(desireAchieved)
            {
                publishTargetGoalInfo(DEL_GOAL_BELIEFS);
                delDesire(targetDesire, true);//desire achieved -> delete all desires within the same group
            }

            if(planExecInfo.status == planExecInfo.SUCCESSFUL)//plan exec completed successful
            {
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                {   
                    string addNote = desireAchieved? 
                        "desire \"" + targetDesireName + "\" achieved will be removed from desire set" : 
                        "desire \"" + targetDesireName + "\" still not achieved! It'll not removed from the desire set yet";
                    
                    RCLCPP_INFO(this->get_logger(), "Plan successfully executed: " + addNote);
                }
            }

            else if(planExecInfo.status == planExecInfo.ABORT && !desireAchieved)// plan exec aborted and desire not achieved
            {

                int maxPlanExecAttempts = this->get_parameter(PARAM_MAX_TRIES_EXEC_PLAN).as_int();
                aborted_plan_desire_map_[targetDesireName]++;
                
                RCLCPP_INFO(this->get_logger(), "Plan execution for fulfilling desire \"" + targetDesireName + 
                    "\" has been aborted for the %d time (max attempts: %d)", 
                        aborted_plan_desire_map_[targetDesireName], maxPlanExecAttempts);
                
                if(aborted_plan_desire_map_[targetDesireName] >= maxPlanExecAttempts)
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Desire \"" + targetDesireName + "\" will be removed because it doesn't seem feasible to fulfill it: too many plan abortions!");
                    delDesire(targetDesire, true);
                
                }else if(!targetDesire.getContext().isSatisfied(belief_set_) && this->get_parameter(PARAM_AUTOSUBMIT_CONTEXT).as_bool()){
                    // check for context condition failed 
                    // (just if not already done... that's why you look into the invalid map)
                    // plan exec could have failed cause of them: evaluate if they can be reached and submit the desire to yourself
                    string fulfillContextDesireName = targetDesire.getName() + "_fulfill_context";
                    
                    // extract a desire (if possible) for each clause in the context conditions
                    vector<ManagedDesire> fulfillContextDesires = BDIFilter::conditionsToMGDesire(targetDesire.getContext(), 
                        fulfillContextDesireName, 
                        std::min(targetDesire.getPriority()+0.01f, 1.0f), targetDesire.getDeadline());
                    
                    for(ManagedDesire fulfillContextD : fulfillContextDesires)
                    {
                        if(desire_set_.count(fulfillContextD) == 0 && desireAcceptanceCheck(fulfillContextD) == ACCEPTED)
                        {
                            if(this->get_parameter(PARAM_DEBUG).as_bool())
                                RCLCPP_INFO(this->get_logger(), "Context conditions are not satisfied for desire \"" + targetDesire.getName() + "\" but could be satisfied: " +  
                                    +  " auto-submission desire \"" + fulfillContextD.getName() + "\"");
                            fulfillContextD.setParent(targetDesire);//set md as its parent desire
                            addDesire(fulfillContextD, targetDesire, "_context");
                        }
                    }
                    
                }

                //  if not reached max exec attempt, for now mantain the desire
                //  if not valid anymore, it'll be eventually removed in next reschedulings, 
                //  otherwise the plan will be commissioned again until reaching maxPlanExecAttempts

                    
            }

            mtx_iter_dset_.unlock();
            current_plan_ = BDIManaged::ManagedPlan{}; // execution has been terminated, current plan empty
            reschedule();
            //next reschedule() will select a new plan if computable for a desire in desire set
        }
    }
}

/*  Use the updated belief set for deciding if some desires are pointless to pursue given the current 
    beliefs which shows they're already fulfilled
*/
void SchedulerOffline::checkForSatisfiedDesires()
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

                    //abort current plan execution since current target desire is already achieved and you're far from completing the plan (missing more than last action)
                    abortCurrentPlanExecution();   
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
    wrt the current plan execution...
    return sum of progress status of all actions within a plan divided by the number of actions
*/
float SchedulerOffline::computePlanProgressStatus()
{   
    // not exeuting any plan or last update not related to currently triggered plan
    if(noPlanExecuting() || current_plan_exec_info_.target.name != current_plan_.getFinalTarget().getName())
        return 0.0f;

    float progress_sum = 0.0f;
    for(auto currentExecutingAction : current_plan_exec_info_.actions_exec_info)
        progress_sum += currentExecutingAction.progress;

    return progress_sum/(current_plan_exec_info_.actions_exec_info.size());
}



/*
    Specific behaviour of scheduler after desire successful addition, based on its selected mode    
*/
void SchedulerOffline::postAddDesireSuccess(const BDIManaged::ManagedDesire& md)
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
void SchedulerOffline::postDelDesireSuccess(const BDIManaged::ManagedDesire& md)
{
    //Offline mode behaviour
    if(md == current_plan_.getFinalTarget())//deleted desire of current executing plan)
        abortCurrentPlanExecution();//abort current plan execution
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); 
  auto node = std::make_shared<SchedulerOffline>();
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
