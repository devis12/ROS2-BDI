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
using std::queue;
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

using javaff_interfaces::msg::PartialPlans;

using BDIManaged::ManagedDesire;
using BDIManaged::ManagedPlan;


void SchedulerOnline::init()
{
    Scheduler::init();

    //init SchedulerOffline specific props
    fulfilling_desire_ = ManagedDesire{};
    waiting_plans_ = queue<ManagedPlan>();

    searching_ = false;

    // interval search ms
    this->declare_parameter(JAVAFF_SEARCH_INTERVAL_PARAM, JAVAFF_SEARCH_INTERVAL_PARAM_DEFAULT);

    javaff_client_ = std::make_shared<JavaFFClient>(string("javaff_srvs_caller"));

    javaff_pplans_subscriber_ = this->create_subscription<PartialPlans>(
        JAVAFF_PPLANS_TOPIC, 10,
            bind(&SchedulerOnline::updatedIncrementalPlan, this, _1));

}

/*
    Check whether a plan is executing
*/
bool SchedulerOnline::noPlanExecuting()
{
    return fulfilling_desire_.getValue().size() == 0 && waiting_plans_.size() == 0;
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

    RCLCPP_INFO(this->get_logger(), "Starting search for the fullfillment of Alex's desire to " + selDesire.getName());
    

    if(selDesire.getValue().size() > 0 && launchPlanSearch(selDesire))//a desire has effectively been selected && a search for it has been launched
    {    
        searching_ = true;
        RCLCPP_INFO(this->get_logger(), "Search started for the fullfillment of Alex's desire to " + selDesire.getName());
        fulfilling_desire_ = selDesire; 
    }
}

/*
    Received update on current plan execution
*/
void SchedulerOnline::updatePlanExecution(const BDIPlanExecutionInfo::SharedPtr msg)
{
    
}

/*
    Received update on current plan search
*/
void SchedulerOnline::updatedIncrementalPlan(const javaff_interfaces::msg::PartialPlans::SharedPtr msg)
{
    if(searching_)
    {
        // TODO store incremental partial plans 
        // as soon as you have the first, trigger plan execution
        string pplans = "";
        for(auto pplan : msg->plans)
        {
            pplans += "\n\n-\n";
            for(auto pplanItem : pplan.items)
                pplans += "\t[" + std::to_string(pplanItem.time) + ", " + std::to_string(pplanItem.duration) + "]: \t" + pplanItem.action  + "\n";
        }
        RCLCPP_INFO(this->get_logger(), pplans);
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
    return javaff_client_->launchPlanSearch(pddl_problem, intervalSearchMS);
}

/*
    wrt the current plan execution...
    return sum of progress status of all actions within a plan divided by the number of actions
*/
float SchedulerOnline::computePlanProgressStatus()
{   
    // not exeuting any plan or last update not related to currently triggered plan
    if(noPlanExecuting() || current_plan_exec_info_.target.name != waiting_plans_.front().getDesire().getName())
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
            if(!noPlanExecuting() && waiting_plans_.front().getDesire() == md && 
                current_plan_exec_info_.status == current_plan_exec_info_.RUNNING)  
            {
                float plan_progress_status = computePlanProgressStatus();
                
                if(plan_progress_status < COMPLETED_THRESHOLD)
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Current plan execution fulfilling desire \"" + md.getName() + 
                            "\" will be aborted since desire is already fulfilled and plan exec. is still far from being completed " +
                            "(progress status = %f)", plan_progress_status);
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
    if(md == waiting_plans_.front().getDesire())//deleted desire of current executing plan)
    {
        //TODO ABORT CURRENT AND WAITING PLANS, still has to be understood
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
