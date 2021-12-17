#ifndef PLAN_DIRECTOR_H_
#define PLAN_DIRECTOR_H_

#include <string>
#include <vector>
#include <set>
#include <optional>
#include <memory>
#include <chrono>


#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/plan_sys2_state.hpp"
#include "ros2_bdi_interfaces/msg/bdi_action_execution_info.hpp"
#include "ros2_bdi_interfaces/msg/bdi_plan_execution_info.hpp"

#include "ros2_bdi_interfaces/srv/bdi_plan_execution.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedPlan.hpp"

#include "rclcpp/rclcpp.hpp"

typedef enum {STARTING, READY, EXECUTING, PAUSE} StateType;                

class PlanDirector : public rclcpp::Node
{
public:
  
    /* Constructor method */
    PlanDirector();

    /*
        Init to call at the start, after construction method, to get the node actually started
        initialing psys2 executor client instance, 
        retrieving agent_id_ (thus namespace)
        defining work timer
    */
    void init();
    
    /*
        Main loop of work called regularly through a wall timer
    */
    void step();

private:
    /*  
        Change internal state of the node
    */
    void setState(StateType state){ state_ = state;  }

    // clear info about current plan execution
    void setNoPlanMsg(){ current_plan_ = BDIManaged::ManagedPlan{}; }

    /*
        Check planExecution feedback from plansys2 executor + check for context conditions
    */
    void executingPlan();

    /*
        When in READY state, msg to publish in plan_execution_info to notify it 
        (i.e. notify you're not executing any plan)
    */
    void publishNoPlanExec();
    
    /*
       Received notification about PlanSys2 nodes state by plansys2 monitor node
    */
    void callbackPsys2State(const ros2_bdi_interfaces::msg::PlanSys2State::SharedPtr msg);

    /*
        Currently executing no plan
    */
    bool executingNoPlan();

    /*
        Cancel current plan execution (if any) and information preserved in it
    */
    void cancelCurrentPlanExecution();


    /*
        Start new plan execution -> true if correctly started
    */
    bool startPlanExecution(const BDIManaged::ManagedPlan& mp);



    // redefine workint timer interval
    void resetWorkTimer(const int& ms);

    /*
        Returns all the items within a PlanItem.action string
        E.g. "(dosweep sweeper kitchen)" -> ["dosweep", "sweeper", "kitchen"]
    */
    std::vector<std::string> extractPlanItemActionElements(std::string planItemAction);

    /*
        Return true if plan exec request is well formed 
            - request = ABORT | EXECUTE
            - at least a planitem elem in body
            - for each plan item action
                - check its definition exists with domain expert
                - check its params are valid instances (problem expert) matching the correct types (domain expert)
    */
    bool validPlanRequest(const ros2_bdi_interfaces::srv::BDIPlanExecution::Request::SharedPtr request);

    /*  
        Callback to handle the service request to trigger a new plan execution or abort the current one
    */
    void handlePlanRequest(const ros2_bdi_interfaces::srv::BDIPlanExecution::Request::SharedPtr request,
        const ros2_bdi_interfaces::srv::BDIPlanExecution::Response::SharedPtr response);

    /*
        Plan currently in execution, monitor and publish the feedback of its development
    */
    void checkContextConditions();

    /* 
        publish beliefs to be added and beliefs to be deleted as a consequence of plan abortion (rollback)
    */
    void publishRollbackBeliefs(const std::vector<ros2_bdi_interfaces::msg::Belief> rollback_belief_add, 
        const std::vector<ros2_bdi_interfaces::msg::Belief> rollback_belief_del);


    /*
        Plan currently in execution, monitor and publish the feedback of its development
    */
    void checkPlanExecution();


    ros2_bdi_interfaces::msg::BDIPlanExecutionInfo getPlanExecutionInfo(const plansys2::ExecutorClient::ExecutePlan::Feedback& feedback);



    uint8_t getPlanExecutionStatus();


    /*  
        Take time msg (int second, int nanosecond)
        transform it in float second
    */
    float fromTimeMsgToSeconds(const int& sec_ts, const int& nanosec_ts);

    /*
        get index of action with given action_name & args in the vector<PlanItem> in current_plan_.body 
    */
    int getActionIndex(const std::string& action_name, const std::vector<std::string>& args);

    /*
        The belief set has been updated
    */
    void updatedBeliefSet(const ros2_bdi_interfaces::msg::BeliefSet::SharedPtr msg);

    // internal state of the node
    StateType state_;
    
    // agent id that defines the namespace in which the node operates
    std::string agent_id_;

    // timer to trigger callback to perform main loop of work regularly
    rclcpp::TimerBase::SharedPtr do_work_timer_;

    // counter of communication errors with plansys2
    int psys2_comm_errors_;
    // domain expert client contacting psys2 for checking validity of a plan
    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_client_;
    // problem expert client contacting psys2 for checking validity of a plan
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_client_;
    // executor client contacting psys2 for the execution of a plan, then receiving feedback for it 
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;

    // flag to denote if plansys2 domain expert appears to be active
    bool psys2_domain_expert_active_;
    // flag to denote if plansys2 problem expert appears to be active
    bool psys2_problem_expert_active_;
    // flag to denote if plansys2 executor appears to be active
    bool psys2_executor_active_;
    // plansys2 node status monitor subscription
    rclcpp::Subscription<ros2_bdi_interfaces::msg::PlanSys2State>::SharedPtr plansys2_status_subscriber_;

    // current_plan_ in execution (could be none if the agent isn't doing anything)
    BDIManaged::ManagedPlan current_plan_;
    // # checks performed during the current plan exec
    int counter_check_;
    // time at which plan started (NOT DOING this anymore -> using first start_ts from first action executed in plan)
    //high_resolution_clock::time_point current_plan_start_;
    // msg to notify the idle-ready state, i.e. no current plan execution, but ready to do it
    ros2_bdi_interfaces::msg::BDIPlanExecutionInfo no_plan_msg_;

    // current belief set (in order to check precondition && context condition)
    std::set<BDIManaged::ManagedBelief> belief_set_;
    // belief set subscriber
    rclcpp::Subscription<ros2_bdi_interfaces::msg::BeliefSet>::SharedPtr belief_set_subscriber_;//belief set sub.
    // belief add publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr belief_add_publisher_;
    // belief del publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr belief_del_publisher_;

    // record first timestamp in sec of the current plan execution (to subtract from it)
    int first_ts_plan_sec;
    int first_ts_plan_nanosec;
    // last recorded timestamp during plan execution
    float last_ts_plan_exec;

    // notification about the current plan execution -> plan execution info publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::BDIPlanExecutionInfo>::SharedPtr plan_exec_publisher_;

    // trigger plan execution/abortion service
    rclcpp::Service<ros2_bdi_interfaces::srv::BDIPlanExecution>::SharedPtr server_plan_exec_;

}; // PlanDirector class prototype

#endif // PLAN_DIRECTOR