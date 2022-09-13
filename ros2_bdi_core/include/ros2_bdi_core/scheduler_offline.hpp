#ifndef SCHEDULER_OFFLINE_H_
#define SCHEDULER_OFFLINE_H_

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
        Select plan execution based on precondition, deadline
    */
    void reschedule();

    /*
        Received update on current plan execution
    */
    void updatePlanExecution(const ros2_bdi_interfaces::msg::BDIPlanExecutionInfo::SharedPtr msg);

    /*
        wrt the current plan execution...
        return sum of progress status of all actions within a plan divided by the number of actions
    */
    float computePlanProgressStatus();
};

#endif // SCHEDULER_OFFLINE_H_