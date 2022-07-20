#include "ros2_bdi_core/scheduler.hpp"
#include "ros2_bdi_core/support/javaff_client.hpp"

#include "javaff_interfaces/msg/search_result.hpp"

#include "ros2_bdi_utils/BDIPlanLibrary.hpp"

#include "rclcpp/rclcpp.hpp"


class SchedulerOnline : public Scheduler
{
public:
    SchedulerOnline() : Scheduler(), 
        planlib_db_(PlanLibrary::BDIPlanLibrary{"/tmp/"+agent_id_+"/"+PLAN_LIBRARY_NAME}){};

    void init() override;


private:
    
    /*
        Store plan in plan library && enqueue in waiting_plans
    */
    void storeEnqueuePlan(BDIManaged::ManagedPlan&mp);

    /*
        Store plan in plan library
    */
    void storePlan(BDIManaged::ManagedPlan&mp);

    /*
        Enqueue plan in waiting list for execution
    */
    void enqueuePlan(BDIManaged::ManagedPlan& el)
    {
        if(waiting_plans_.size() == 0)
            waiting_plans_.push_back(el);
        else
            waiting_plans_.insert(waiting_plans_.begin(), el);
    }

    /*
        Dequeue plan from execution waiting list 
    */
    std::optional<BDIManaged::ManagedPlan> dequeuePlan()
    {
        if(waiting_plans_.size() == 0)
            return std::nullopt;

        BDIManaged::ManagedPlan el = waiting_plans_.back();
        waiting_plans_.pop_back();
        return el;
    }

    /*
        Return first plan to be executed in waiting list
    */
    std::optional<BDIManaged::ManagedPlan> waitingPlansFront()
    {
        if(waiting_plans_.size() == 0)
            return std::nullopt;
            
        return waiting_plans_[waiting_plans_.size() -1];
    }

    /*
        Return last plan to be executed in waiting list
    */
    std::optional<BDIManaged::ManagedPlan> waitingPlansBack()
    {        
        if(waiting_plans_.size() == 0)
            return std::nullopt;
        return waiting_plans_[0];
    }

    /*
        Return last plan to be executed in waiting list
    */
    bool replaceLastPlan(const BDIManaged::ManagedPlan& mp)
    {        
        if(waiting_plans_.size() == 0)
            return false;
            
        waiting_plans_[0] = mp;//first element of the vector is the last of the queue since the items are inserted from the top
        return true;
    }

    /*
        Find plan index, -1 if not there
    */
    bool findPlanIndex(const BDIManaged::ManagedPlan& mp)
    {        
        for(int i=0; i<waiting_plans_.size(); i++)
            if(mp == waiting_plans_[i])
                return i;
        return -1;
    }

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
        Init all info related to current desire in pursuit & plan to fulfill it, then launch reschedule
    */
    void forcedReschedule();
    
    /*
        wrt the current plan execution...
        return sum of progress status of all actions within a plan divided by the number of actions
    */
    float computePlanProgressStatus();


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
    void updatedIncrementalPlan(const javaff_interfaces::msg::SearchResult::SharedPtr msg);

    /*
        Call JavaFF for triggering the search for a plan fulfilling @selDesire
    */
    bool launchPlanSearch(const BDIManaged::ManagedDesire selDesire);

    // queue of waiting_plans for execution (LAST element of the vector is the first one that has been pushed)
    std::vector<BDIManaged::ManagedPlan> waiting_plans_;

    // fulfilling desire 
    BDIManaged::ManagedDesire fulfilling_desire_;

    // search is progressing
    bool searching_;

    // computed partial plans echoed by JavaFF
    rclcpp::Subscription<javaff_interfaces::msg::SearchResult>::SharedPtr javaff_search_subscriber_;//javaff search sub.

    // Client to wrap srv call to JavaFFServer
    std::shared_ptr<JavaFFClient> javaff_client_;

    //Plan library db utility (just used in online for now: put it here, so we can close connection in bringdown)
    PlanLibrary::BDIPlanLibrary planlib_db_;
    bool planlib_conn_ok_;

    // Index of executing partial plan in the queue of executions for current global target in fulfillment 
    int executing_pplan_index_;
};
