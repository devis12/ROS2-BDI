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

using BDIManaged::ManagedDesire;
using BDIManaged::ManagedPlan;


void SchedulerOnline::init()
{
    Scheduler::init();
}

/*
    Check whether a plan is executing
*/
bool SchedulerOnline::noPlanExecuting()
{
    return waiting_plans_.size() == 0;
}

/*
    Select plan execution based on precondition, deadline
*/
void SchedulerOnline::reschedule()
{   
    RCLCPP_INFO(this->get_logger(), "Online rescheduling");
}

/*
    Received update on current plan execution
*/
void SchedulerOnline::updatePlanExecution(const BDIPlanExecutionInfo::SharedPtr msg)
{
    
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

    if(desire_set_.size() > 0 && noPlanExecuting())// still there to be satisfied && no plan selected, rescheduled immediately
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
