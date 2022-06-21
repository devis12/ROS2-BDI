#include <queue>

#include "ros2_bdi_core/scheduler.hpp"

#include "javaff_interfaces/msg/partial_plans.hpp"

#include "rclcpp/rclcpp.hpp"


class SchedulerOnline : public Scheduler
{
public:
    SchedulerOnline() : Scheduler() {};

    void init() override;

private:
    /*
        Specific behaviour of scheduler after desire successful addition, based on its selected mode    
    */
    void postAddDesireSuccess(const BDIManaged::ManagedDesire& md);

    /*
        Specific behaviour of scheduler after desire successful deletion, based on its selected mode    
    */
    void postDelDesireSuccess(const BDIManaged::ManagedDesire& md);

    /*
        Operations related to the selection of the next active desire and Intentions to be enforced
    */
    void reschedule();
    
    /*
        wrt the current plan execution...
        return sum of progress status of all actions within a plan divided by the number of actions
    */
    float computePlanProgressStatus();

    /*
        Check whether a plan is executing
    */
    bool noPlanExecuting();

    /*  Use the updated belief set for deciding if some desires are pointless to pursue given the current 
        beliefs which shows they're already fulfilled
    */
    void checkForSatisfiedDesires();

    /*
        Received update on current plan execution
    */
    void updatePlanExecution(const ros2_bdi_interfaces::msg::BDIPlanExecutionInfo::SharedPtr msg);

    /*
        Received update on current plan search
    */
    void updatedIncrementalPlan(const javaff_interfaces::msg::PartialPlans::SharedPtr msg);

    // waiting_plans for execution
    std::queue<BDIManaged::ManagedPlan> waiting_plans_;

    // fulfilling desire 
    BDIManaged::ManagedDesire fulfilling_desire_;

    // computed partial plans echoed by JavaFF
    rclcpp::Subscription<javaff_interfaces::msg::PartialPlans>::SharedPtr javaff_pplans_subscriber_;//javaff pplans sub.
};
