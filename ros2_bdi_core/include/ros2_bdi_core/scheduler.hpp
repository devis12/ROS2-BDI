#include <optional>
#include <mutex>
#include <vector>
#include <set>   
#include <map>   

#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_interfaces/msg/condition.hpp"
#include "ros2_bdi_interfaces/msg/conditions_conjunction.hpp"
#include "ros2_bdi_interfaces/msg/conditions_dnf.hpp"
#include "ros2_bdi_interfaces/msg/planning_system_state.hpp"
#include "ros2_bdi_interfaces/msg/bdi_action_execution_info.hpp"
#include "ros2_bdi_interfaces/msg/bdi_plan_execution_info.hpp"
#include "ros2_bdi_interfaces/srv/bdi_plan_execution.hpp"

//#include "javaff_interfaces/msg/partial_plans.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/ManagedPlan.hpp"

#include "ros2_bdi_core/params/core_common_params.hpp"
#include "ros2_bdi_core/params/scheduler_params.hpp"

#include "ros2_bdi_interfaces/msg/lifecycle_status.hpp"
#include "ros2_bdi_core/support/plansys_monitor_client.hpp"
#include "ros2_bdi_core/support/planning_mode.hpp"
#include "ros2_bdi_core/support/trigger_plan_client.hpp"

#include "rclcpp/rclcpp.hpp"

typedef enum {STARTING, SCHEDULING, PAUSE} StateType;          
typedef enum {ACCEPTED, UNKNOWN_PREDICATE, SYNTAX_ERROR, UNKNOWN_INSTANCES} TargetBeliefAcceptance;

typedef enum {ADD_GOAL_BELIEFS, DEL_GOAL_BELIEFS} GoalBeliefOp;

class Scheduler : public rclcpp::Node
{
public:
    Scheduler();

    /*
        Init to call at the start, after construction method, to get the node actually started
        initialing planner client instance, 
        retrieving agent_id_ (thus namespace)
        defining work timer,
        belief set subscriber callback,
        desire set publisher,
        add/del desire subscribers callback
    */
    virtual void init();
    
    /*
        Main loop of work called regularly through a wall timer
    */
    void step();

    /*
        Wait for PlanSys2 to boot at best for max_wait
    */
    bool wait_psys2_boot(const std::chrono::seconds max_wait = std::chrono::seconds(16))
    {
        psys_monitor_client_ = std::make_shared<PlanSysMonitorClient>(SCHEDULER_NODE_NAME + std::string("_psys2caller_"), sel_planning_mode_);
        return psys_monitor_client_->areAllPsysNodeActive(max_wait);
    }

protected:
    /*
        Specific behaviour of scheduler after desire successful addition, based on its selected mode    
    */
    virtual void postAddDesireSuccess(const BDIManaged::ManagedDesire& md) = 0;

    /*
        Specific behaviour of scheduler after desire successful deletion, based on its selected mode    
    */
    virtual void postDelDesireSuccess(const BDIManaged::ManagedDesire& md) = 0;

    /*
        Operations related to the selection of the next active desire and Intentions to be enforced
    */
    virtual void reschedule() = 0;

    /*  Use the updated belief set for deciding if some desires are pointless to pursue given the current 
        beliefs which shows they're already fulfilled
    */
    virtual void checkForSatisfiedDesires() = 0;

    /*
        Received update on current plan execution
    */
    virtual void updatePlanExecution(const ros2_bdi_interfaces::msg::BDIPlanExecutionInfo::SharedPtr msg) = 0;


    /*  
        Change internal state of the node
    */
    void setState(StateType state) { state_ = state;  }

    /*
       Publish the current desire set of the agent in agent_id_/desire_set topic
    */
    void publishDesireSet();


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
        Check if there is a current valid plan selected
    */
    bool noPlanExecuting()
    {
        return current_plan_.getPlanTarget().getPriority() == 0.0f && current_plan_.getActionsExecInfo().size() == 0;
    }


    /*Build updated ros2_bdi_interfaces::msg::LifecycleStatus msg*/
    ros2_bdi_interfaces::msg::LifecycleStatus getLifecycleStatus();

    /*
       Received notification about PlanSys2 nodes state by plansys2 monitor node
    */
    void callbackPsys2State(const ros2_bdi_interfaces::msg::PlanningSystemState::SharedPtr msg);

    /*
        Received notification about ROS2-BDI Lifecycle status
    */
    void callbackLifecycleStatus(const ros2_bdi_interfaces::msg::LifecycleStatus::SharedPtr msg)
    {
        if(lifecycle_status_.find(msg->node_name) != lifecycle_status_.end())//key in map, record upd value
            lifecycle_status_[msg->node_name] = msg->status;
    }

    /*
        Expect to find yaml file to init the desire set in "/tmp/{agent_id}/init_dset.yaml"
    */
    void tryInitDesireSet();

    /*
        returns ACCEPTED iff managed belief can be put as part of a desire's value
        wrt. to its syntax
    */
    TargetBeliefAcceptance targetBeliefAcceptanceCheck(const BDIManaged::ManagedBelief& mb);

    /*
         Check with the domain_expert and problem_expert to understand if this is a valid goal
        (i.e. valid predicates and valid instances defined within them)

        returns true iff managed desire can be considered syntactically correct
        (no syntactically incorrect beliefs)
    */
    TargetBeliefAcceptance desireAcceptanceCheck(const BDIManaged::ManagedDesire& md);

    /*
        Publish target goal info to belief set
    */
    void publishTargetGoalInfo(const GoalBeliefOp& op);

    /*
        Given the current knowledge of the belief set, decide if a given desire
        is already fulfilled
    */
    bool isDesireSatisfied(BDIManaged::ManagedDesire& md);

    /*
        The belief set has been updated
    */
    void updatedBeliefSet(const ros2_bdi_interfaces::msg::BeliefSet::SharedPtr msg);

    /*  
        Someone has publish a new desire to be fulfilled in the respective topic
    */
    void addDesireTopicCallBack(const ros2_bdi_interfaces::msg::Desire::SharedPtr msg);

    /*  
        Someone has publish a desire to be removed from the one to be fulfilled (if present)
        in the respective topic
    */
    void delDesireTopicCallBack(const ros2_bdi_interfaces::msg::Desire::SharedPtr msg);

    /*
        Wrapper for calling addDesire with just desire to added (where not linked to any other desires)
        N.B see addDesire(const ManagedDesire mdAdd, const optional<ManagedDesire> necessaryForMd)
        for further explanations
    */
    bool addDesire(const BDIManaged::ManagedDesire mdAdd)
    {
        return addDesire(mdAdd, std::nullopt, "");
    }

    /*
        Add desire @mdAdd to desire_set_ (if there is not yet)
        add counters for invalid desire and aborted plan in respective maps (both init to zero)
        @necessaryForMd is another ManagedDesire which can be present if @mdAdd is necessary 
        for the fulfillment of it (e.g. @mdAdd is derived from preconditions or context)
        @necessaryForMd value (if exists) has to be already in the desire_set_
    */
    bool addDesire(BDIManaged::ManagedDesire mdAdd, std::optional<BDIManaged::ManagedDesire> necessaryForMD, 
        const std::string& suffixDesireGroup)
    {   
        auto acceptance = desireAcceptanceCheck(mdAdd);
        if(acceptance != ACCEPTED && acceptance != UNKNOWN_INSTANCES)//unknown predicates values / syntax_errors within target beliefs
            return false;

        bool added = false;
        mtx_add_del_.lock();
            added = addDesireCS(mdAdd, necessaryForMD, suffixDesireGroup);
        mtx_add_del_.unlock();
        return added;
    }

    /*
        Add desire Critical Section (to be executed AFTER having acquired mtx_add_del_.lock())

        Add desire @mdAdd to desire_set_ (if there is not yet)
        add counters for invalid desire and aborted plan in respective maps (both init to zero)
        @necessaryForMd is another ManagedDesire which can be present if @mdAdd is necessary 
        for the fulfillment of it (e.g. @mdAdd is derived from preconditions or context)
        @necessaryForMd value (if exists) has to be already in the desire_set_
    */
    bool addDesireCS(BDIManaged::ManagedDesire mdAdd, std::optional<BDIManaged::ManagedDesire> necessaryForMD, 
        const std::string& suffixDesireGroup);
    
    /*
        Wrapper for delDesire with two args
    */
    bool delDesire(const BDIManaged::ManagedDesire mdDel)
    {
        return delDesire(mdDel, false);//do not delete desires within the same group of mdDel
    }

    /*
        Del desire from desire_set if present (Access through lock!)
    */
    bool delDesire(const BDIManaged::ManagedDesire mdDel, const bool& wipeSameGroup)
    {
        bool deleted = false;
        mtx_add_del_.lock();
            deleted = delDesireCS(mdDel, wipeSameGroup);
        mtx_add_del_.unlock();

        return deleted;
    }
    
    /*
        Del desire from desire_set CRITICAL SECTION (to be called after having acquired mtx_add_del_ lock)
        In Addition Deleting atomically all the desires within the same desire group
        if @wipeSameGroup equals to true
    */
    bool delDesireCS(const BDIManaged::ManagedDesire mdDel, const bool& wipeSameGroup);

    /*
        Del desire group from desire_set CRITICAL SECTION (to be called after having acquired mtx_add_del_ lock)
        Deleting atomically all the desires within the same desire group
    */
    void delDesireInGroupCS(const std::string& desireGroup);

    //void reschedulingOnline();

    // internal state of the node
    StateType state_;

    // step counter
    uint64_t step_counter_;

    // Selected planning mode
    PlanningMode sel_planning_mode_;

    // current_plan in execution
    BDIManaged::ManagedPlan current_plan_;
    
    // agent id that defines the namespace in which the node operates
    std::string agent_id_;
    // callback to perform main loop of work regularly
    rclcpp::TimerBase::SharedPtr do_work_timer_;

    // counter of communication errors with plansys2
    int psys2_comm_errors_;
    // problem expert instance to call the plansys2 problem expert api
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    // domain expert instance to call the plansys2 domain expert api
    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    // planner expert instance to call the plansys2 planner api
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    
    // flag to denote if the javaff online planner is up and active
    bool javaff_planner_active_;

    // flag to denote if the plansys2 planner is up and active
    bool psys2_planner_active_;
    // flag to denote if the plansys2 domain expert is up and active
    bool psys2_domain_expert_active_;
    // flag to denote if the plansys2 problem expert planner is up and active
    bool psys2_problem_expert_active_;
    // plansys2 node status monitor subscription
    rclcpp::Subscription<ros2_bdi_interfaces::msg::PlanningSystemState>::SharedPtr plansys2_status_subscriber_;
    
    // desire set has been init. (or at least the process to do so has been tried)
    bool init_dset_;

    // belief set of the agent <agent_id_>
    std::set<BDIManaged::ManagedBelief> belief_set_;

    // desire set of the agent <agent_id_>
    std::set<BDIManaged::ManagedDesire> desire_set_;

    // hashmap for invalid desire or plan not computed counters (after x tries desire will be discarded)
    std::map<std::string, int> computed_plan_desire_map_;
    // hashmap for aborted plan desire map (plan aborted for that desire)
    std::map<std::string, int> aborted_plan_desire_map_;

    // waiting_plans_ in execution + waiting plans for execution (could be none if the agent isn't doing anything)
    //std::queue<BDIManaged::ManagedPlan> waiting_plans_;

    // last plan execution info
    ros2_bdi_interfaces::msg::BDIPlanExecutionInfo current_plan_exec_info_;
    // Plan Execution Service manager for client operations
    std::shared_ptr<TriggerPlanClient> plan_exec_srv_client_;

    //mutex for sync when modifying desire_set
    std::mutex mtx_add_del_;
    //mutex to regulate abort -> trigger new one flow
    //mutex mtx_abort_trigger_;
    //to sync between iteration in checkForSatisfiedDesires() && reschedule()
    std::mutex mtx_iter_dset_;

    // string representing a pddl domain to supply to the plansys2 planner
    std::string pddl_domain_;
    // string representing a pddl problem to supply to the plansys2 planner
    std::string pddl_problem_;

    // desire set publishers/subscribers
    rclcpp::Subscription<ros2_bdi_interfaces::msg::Desire>::SharedPtr add_desire_subscriber_;//add desire notify on topic
    rclcpp::Subscription<ros2_bdi_interfaces::msg::Desire>::SharedPtr del_desire_subscriber_;//del desire notify on topic
    rclcpp::Publisher<ros2_bdi_interfaces::msg::DesireSet>::SharedPtr desire_set_publisher_;//desire set publisher

    // belief set publisher (to publish info wrt. currently active desire)
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr add_belief_publisher_;//add belief publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr del_belief_publisher_;//del belief publisher

    // belief set subscriber
    rclcpp::Subscription<ros2_bdi_interfaces::msg::BeliefSet>::SharedPtr belief_set_subscriber_;//belief set sub.

    // plan executioninfo subscriber
    rclcpp::Subscription<ros2_bdi_interfaces::msg::BDIPlanExecutionInfo>::SharedPtr plan_exec_info_subscriber_;//plan execution info publisher

    // current known status of the system nodes
    std::map<std::string, uint8_t> lifecycle_status_;
    // Publish updated lifecycle status
    rclcpp::Publisher<ros2_bdi_interfaces::msg::LifecycleStatus>::SharedPtr lifecycle_status_publisher_;
    // Sub to updated lifecycle status
    rclcpp::Subscription<ros2_bdi_interfaces::msg::LifecycleStatus>::SharedPtr lifecycle_status_subscriber_;

    // PlanSys2 Monitor Client supporting nodes & clients for calling the {psys2_node}/get_state services
    std::shared_ptr<PlanSysMonitorClient> psys_monitor_client_;
};
