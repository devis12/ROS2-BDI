#ifndef BDI_ACTION_EXECUTOR_H_
#define BDI_ACTION_EXECUTOR_H_

#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>
#include <set>

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

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class BDIActionExecutor : public plansys2::ActionExecutorClient
{
public:
  /*
    Constructor for every action executor node, 
      @action_name should match the one within the pddl domain definition
      @working_freq represents the frequency at which the doWork method is called (expressed in Hz)
  */
  BDIActionExecutor(const std::string& action_name, const int& working_freq);

  /*
    Method called when node is triggered by the Executor node of PlanSys2
    Progress for the action sets to 0.0f
  */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
      on_activate(const rclcpp_lifecycle::State & previous_state);

protected:

    /*returns arguments of the actions*/
    std::vector<std::string> getArguments() {return get_arguments();}
    /*returns current progress state of the action*/
    float getProgress() {return progress_;} 

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
    BDICommunications::CheckBeliefResult sendCheckBeliefRequest(const std::string& agentRef, 
        const ros2_bdi_interfaces::msg::Belief& belief);
    BDICommunications::UpdBeliefResult sendUpdBeliefRequest(const std::string& agentRef, 
        const ros2_bdi_interfaces::msg::Belief& belief, const BDICommunications::UpdOperation& op);
    
    /*
      Utility methods to check other agents' desire sets or 
      send desire request (ADD/DEL) to other agents within the action doWork method implementation
      (flag for monitoring fulfillment is provided for the ADD scenario)
    */
    BDICommunications::CheckDesireResult sendCheckDesireRequest(const std::string& agentRef, 
        const ros2_bdi_interfaces::msg::Desire& desire);
    BDICommunications::UpdDesireResult sendUpdDesireRequest(const std::string& agentRef, 
        const ros2_bdi_interfaces::msg::Desire& desire, const BDICommunications::UpdOperation& op, const bool& monitorFulfill);

    /*
      if no monitored desire, just return false
      otherwise check if it is fulfilled in the respective monitored belief set
    */
    bool isMonitoredDesireSatisfied();

    // method to be called when the execution successfully comes to completion (generic success msg added)
    void execSuccess() { execSuccess(""); }

    // method to be called when the execution successfully comes to completion (specific success msg added)
    void execSuccess(const std::string& statusSpecific)
    {
      if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_INFO(this->get_logger(), "Action execution success: " + statusSpecific);
      finish(true, 1.0, action_name_ + " successful execution" + ((statusSpecific == "")? ": action performed" : ": " + statusSpecific));
    }

    //method to be called when the execution fail (generic error to be given: no specific added)
    void execFailed()
    { 
      execFailed("");
    }

    //method to be called when the execution fail (specific error to be given)
    void execFailed(const std::string& errSpecific)
    {
      if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_ERROR(this->get_logger(), "Action execution failed: " + errSpecific);
      finish(false, progress_, action_name_ + " failed execution" + ((errSpecific == "")? ": generic error" : ": " + errSpecific));
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


    //listen to belief set of agents to know if/when desires are fulfilled
    rclcpp::Subscription<ros2_bdi_interfaces::msg::BeliefSet>::SharedPtr agent_belief_set_subscriber_;

    //currently monitoring desire
    BDIManaged::ManagedDesire monitored_desire_;

    //currently monitoring belief set
    std::set<BDIManaged::ManagedBelief> monitored_beliefset_;

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
};

#endif  // BDI_ACTION_EXECUTOR_H_
