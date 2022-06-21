#include "ros2_bdi_core/scheduler.hpp"

#include "rclcpp/rclcpp.hpp"


class SchedulerOffline : public Scheduler
{
public:
    SchedulerOffline() : Scheduler() {};

    void init() override;

protected:

    /*
        Specific behaviour of scheduler after desire successful addition, based on its selected mode    
    */
    void postAddDesireSuccess(const BDIManaged::ManagedDesire& md);
    /*
        Specific behaviour of scheduler after desire successful deletion, based on its selected mode    
    */
    void postDelDesireSuccess(const BDIManaged::ManagedDesire& md);

    /*  Use the updated belief set for deciding if some desires are pointless to pursue given the current 
        beliefs which shows they're already fulfilled
    */
    void checkForSatisfiedDesires();


private:

    /*
        Compute plan from managed desire, setting its belief array representing the desirable state to reach
        as the goal of the PDDL problem 
    */
    std::optional<plansys2_msgs::msg::Plan> computePlan(const BDIManaged::ManagedDesire& md);

    /*
        Check if there is a current valid plan selected
    */
    bool noPlanExecuting()
    {
        return current_plan_.getDesire().getPriority() == 0.0f && current_plan_.getActionsExecInfo().size() == 0;
    }

    /*
        Select plan execution based on precondition, deadline
    */
    void reschedule();


    /*
        If selected plan fit the minimal requirements for a plan (i.e. not empty body and a desire which is in the desire_set)
        try triggering its execution by srv request to PlanDirector (/{agent}/plan_execution)
    */
    bool tryTriggerPlanExecution(const BDIManaged::ManagedPlan& selectedPlan);

    /*
        Launch execution of selectedPlan; if successful waiting_plans_.access() gets value of selectedPlan
        return true if successful
    */
    bool launchPlanExecution(const BDIManaged::ManagedPlan& selectedPlan);
    
    /*
        Abort execution of first waiting_plans_; if successful waiting_plans_ first element is popped
        return true if successful
    */
    bool abortCurrentPlanExecution();

    /*
        Received update on current plan execution
    */
    void updatePlanExecution(const ros2_bdi_interfaces::msg::BDIPlanExecutionInfo::SharedPtr msg);

    /*
        wrt the current plan execution...
        return sum of progress status of all actions within a plan divided by the number of actions
    */
    float computePlanProgressStatus();


    // current_plan in execution
    BDIManaged::ManagedPlan current_plan_;
};
