#ifndef SCHEDULER_ONLINE_H_
#define SCHEDULER_ONLINE_H_

#include "ros2_bdi_core/scheduler.hpp"
#include "ros2_bdi_core/support/javaff_client.hpp"

#include "javaff_interfaces/msg/committed_status.hpp"
#include "javaff_interfaces/msg/search_result.hpp"
#include "javaff_interfaces/msg/execution_status.hpp"

#include "ros2_bdi_utils/BDIPlanLibrary.hpp"

#include "rclcpp/rclcpp.hpp"

#include "plansys2_executor/ExecutorClient.hpp"


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

    void publishCurrentIntention();

    /* Launch first partial plan execution of a queue of plans which are going to be stored in the waiting list, waiting for their turn */
    bool launchFirstPPlanExecution(const javaff_interfaces::msg::PartialPlan& firstpplan);

    /*
        Increment counter for failed computation of plans that aimed at fulfilling desire x
        DelDesire (and group) in case threshold is reached
    */
    void planCompFailureHandler(const BDIManaged::ManagedDesire& md, const bool handleDelete = true);

    /*
        Increment counter for aborted plans that aimed at fulfilling desire x
        DelDesire (and group) in case threshold is reached
    */
    void abortedPlanHandler(const bool handleDelete = true);

    /*
        Specific behaviour of scheduler after desire successful addition, based on its selected mode    
    */
    void postAddDesireSuccess(const BDIManaged::ManagedDesire& md);

    /*
        Specific behaviour of scheduler after desire successful deletion, based on its selected mode    
    */
    void postDelDesireSuccess(const BDIManaged::ManagedDesire& md);

    /* 
        Request to make an early stop at a certain committed point
        NOTE: committedPlan has to be extracted from current_plan_ otherwise request is going to fail for sure
    */
    bool makeEarlyArrestRequest(const plansys2_msgs::msg::Plan& committedPlan);

    /*
        Operations related to the selection of the next active desire and Intentions to be enforced
    */
    void reschedule();

    /*
        Init all info related to current desire in pursuit & plan to fulfill it, then launch reschedule
    */
    void forcedReschedule();

    /*
        Init all info related to current desire in pursuit & plan to fulfill it
    */
    void resetSearchInfo();
    
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
    void updatedSearchResult(const javaff_interfaces::msg::SearchResult::SharedPtr msg);

    /*
        Received update on current committed status for plan execution
    */
    void updatedCommittedStatus(const javaff_interfaces::msg::CommittedStatus::SharedPtr msg);

    /*
        Regular process of updated search result with same search baseline as previous msgs or "original" status, when search was launched from scratch
    */
    void processIncrementalSearchResult(const javaff_interfaces::msg::SearchResult::SharedPtr msg);

    /*
        Process updated search result presenting a new search baseline compared to previous msgs of the same type
        returns true if new baseline and search results are accepted (i.e. not too late)
    */
    bool processSearchResultWithNewBaseline(const javaff_interfaces::msg::SearchResult::SharedPtr msg);

    /*
        Process desire boost request for active goal augmentation
    */
    void boostDesireTopicCallBack(const ros2_bdi_interfaces::msg::Desire::SharedPtr msg);

    /*
        Call JavaFF for triggering the search for a plan fulfilling @selDesire
    */
    bool launchPlanSearch(const BDIManaged::ManagedDesire& selDesire);

    /* build empty search baseline method */
    javaff_interfaces::msg::CommittedStatus emptySearchBaseline()
    {
        javaff_interfaces::msg::CommittedStatus sb = javaff_interfaces::msg::CommittedStatus{};
        sb.executing_plan_index = -1;
        return sb;
    }

    /* 
        compare passed search baseline with current one
            returns: 
                -1: if sb is less recent
                0:  if sb is at the same level (matching baselines)
                1:  if sb is more recent (more actions committed or greater executing plan index)
    */
    int compareBaseline(javaff_interfaces::msg::CommittedStatus sb);
    
    // queue of waiting_plans for execution (LAST element of the vector is the first one that has been pushed)
    std::vector<BDIManaged::ManagedPlan> waiting_plans_;

    // fulfilling desire 
    BDIManaged::ManagedDesire fulfilling_desire_;

    // search is progressing
    bool searching_;

    // waiting for a clean preemption
    bool waiting_clean_preempt_;

    // committed status of ongoing search result
    javaff_interfaces::msg::CommittedStatus search_baseline_;

    
    rclcpp::Subscription<ros2_bdi_interfaces::msg::Desire>::SharedPtr boost_desire_subscriber_;//boost desire notify on topic

    // computed partial plans echoed by JavaFF
    rclcpp::Subscription<javaff_interfaces::msg::SearchResult>::SharedPtr javaff_search_subscriber_;//javaff search sub.

    // computed next committed state in currently executing plan performed by javaff
    rclcpp::Subscription<javaff_interfaces::msg::CommittedStatus>::SharedPtr executing_plan_committed_status_subscriber_;//javaff committed plan sub.

    // publisher to notify javaff of exec status, needed when active goal is augmented, because pddl problem changes
    rclcpp::Publisher<javaff_interfaces::msg::ExecutionStatus>::SharedPtr javaff_exec_status_publisher_;//javaff exec status pub.

    // Client to wrap srv call to JavaFFServer
    std::shared_ptr<JavaFFClient> javaff_client_;

    //Plan library db utility (just used in online for now: put it here, so we can close connection in bringdown)
    PlanLibrary::BDIPlanLibrary planlib_db_;
    bool planlib_conn_ok_;

    // Index of executing partial plan in the queue of executions for current global target in fulfillment 
    int executing_pplan_index_;
};

#endif // SCHEDULER_ONLINE_H_