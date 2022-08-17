#ifndef BDI_ACTION_EXECUTOR_H_
#define BDI_ACTION_EXECUTOR_H_

#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>
#include <set>
#include <tuple>
#include <map>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"


#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/srv/check_belief.hpp"
#include "ros2_bdi_interfaces/srv/upd_belief_set.hpp"
#include "ros2_bdi_interfaces/srv/check_desire.hpp"
#include "ros2_bdi_interfaces/srv/upd_desire_set.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"

#include "ros2_bdi_skills/communications_structs.hpp"
#include "ros2_bdi_skills/communications_client.hpp"

// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"

#include "javaff_interfaces/msg/action_execution_status.hpp"
#include "javaff_interfaces/msg/execution_status.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

typedef std::tuple
        <
          std::string, 
          BDIManaged::ManagedDesire, 
          rclcpp::Subscription<ros2_bdi_interfaces::msg::BeliefSet>::SharedPtr
        > 

        MonitorDesire;

class BDIActionExecutor : public plansys2::ActionExecutorClient
{
public:
  /*
    Constructor for every action executor node, 
      @action_name should match the one within the pddl domain definition
      @working_freq represents the frequency at which the doWork method is called (expressed in Hz)
  */
  BDIActionExecutor(const std::string action_name, const int working_freq, const bool agent_id_as_specialized_arg = true);

  /*
    Method called when node is triggered by the Executor node of PlanSys2
    Progress for the action sets to 0.0f
  */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
      on_activate(const rclcpp_lifecycle::State & previous_state);

  /*
    Method called when node is deactivate by the Executor node of PlanSys2
    cleanup of monitored_desires, subscription, belief set if needed
  */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
      on_deactivate(const rclcpp_lifecycle::State & previous_state);


protected:

    /*returns arguments of the actions*/
    std::vector<std::string> getArguments() {return get_arguments();}
    /*returns current progress state of the action*/
    float getProgress() {return progress_;} 

    /* Communicate execution status*/
    void communicateExecStatus(const uint8_t& status);

    /*returns full action name*/
    std::string getFullActionName(const bool& withParenthesis=true)
    {
      std::string argsJoin = "";
          for(auto arg : getArguments())
              argsJoin+= " " + arg;
      std::string fullName = get_action_name() + argsJoin; 
      return withParenthesis? "(" + fullName + ")" : fullName;
    }

    /*
      Method regularly called at the frequency specified in the constructor to handle the advancement of the action,
      wrapper method for the domain specific logics written within advanceWork by the user. Its purpose is to hide
      calls that has to be made to PlanSys2, to avoid the user the burden to check a second docs.
    */
    void do_work();

    /*
      Method called once in each run of do_work() with the domain specific logic for the action advancement 
      written by the user. Returns the estimated percentage of progress made in the [0-1] range
    */
    virtual float advanceWork() = 0;

    /*
      Utility methods to check other agents' desire sets or 
      to send belief request (ADD/DEL) to other agents within the action doWork method implementation
    */
    BDICommunications::CheckBeliefResult sendCheckBeliefRequest(const std::string& agent_ref, 
        const ros2_bdi_interfaces::msg::Belief& belief);
    BDICommunications::UpdBeliefResult sendUpdBeliefRequest(const std::string& agent_ref, 
        const ros2_bdi_interfaces::msg::Belief& belief, const BDICommunications::UpdOperation& op);
    
    /*
      Utility methods to check other agents' desire sets or 
      send desire request (ADD/DEL) to other agents within the action doWork method implementation
      (flag for monitoring fulfillment is provided for the ADD scenario)
    */
    BDICommunications::CheckDesireResult sendCheckDesireRequest(const std::string& agent_ref, 
        const ros2_bdi_interfaces::msg::Desire& desire);
    BDICommunications::UpdDesireResult sendUpdDesireRequest(const std::string& agent_ref, 
        const ros2_bdi_interfaces::msg::Desire& desire, const BDICommunications::UpdOperation& op, const bool& monitor_fulfill);

    /*
      if no monitored desire, just return false
      otherwise check if it is fulfilled in the respective monitored belief set
    */
    bool isMonitoredDesireFulfilled(const std::string& agent_ref, const ros2_bdi_interfaces::msg::Desire& desire);

    // method to be called when the execution successfully comes to completion (generic success msg added)
    void execSuccess() { execSuccess(""); }

    // method to be called when the execution successfully comes to completion (specific success msg added)
    void execSuccess(const std::string& success_log)
    {
      communicateExecStatus(javaff_interfaces::msg::ActionExecutionStatus().SUCCESS);

      if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_INFO(this->get_logger(), "Action execution success: " + success_log);
      finish(true, 1.0, action_name_ + " successful execution" + ((success_log == "")? ": action performed" : ": " + success_log));
    }

    //method to be called when the execution fail (generic error to be given: no specific added)
    void execFailed()
    { 
      execFailed("");
    }

    //method to be called when the execution fail (specific error to be given)
    void execFailed(const std::string& err_log)
    {
      if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_ERROR(this->get_logger(), "Action execution failed: " + err_log);
      finish(false, progress_, action_name_ + " failed execution" + ((err_log == "")? ": generic error" : ": " + err_log));
    }


private:

    /*
      Monitor belief set update of an agent 
    */
    void monitor(const std::string& agentRef, const ros2_bdi_interfaces::msg::Desire& desire);

    /*
      update the current monitored belief set 
    */
    void agentBeliefSetCallback(const ros2_bdi_interfaces::msg::BeliefSet::SharedPtr msg);


    //currently monitored desires: vector of tuples in the form (agent_id, desire to fulfill, subs to belief set of agent_id)
    std::vector<MonitorDesire>  monitored_desires_;

    //currently monitoring belief sets: map (agent_id, belief set for agent_id)
    std::map<std::string, std::set<BDIManaged::ManagedBelief>> monitored_bsets_;

    // action name
    std::string action_name_;

    // action params
    std::vector<std::string> action_params_;

    // agent id that defines the namespace in which the node operates
    std::string agent_id_;

    // agent group id that defines the group of agents it is a part of (used to decide which belief/desire to accept or discard)
    std::string agent_group_;

    std::shared_ptr<BDICommunications::CommunicationsClient> comm_client_;

    // progress of the current action execution
    float progress_;

    // Publish updated exec action status to online planner
    rclcpp_lifecycle::LifecyclePublisher<javaff_interfaces::msg::ExecutionStatus>::SharedPtr exec_status_to_planner_publisher_;

    // problem expert instance to call the problem expert api
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
};

#endif  // BDI_ACTION_EXECUTOR_H_
