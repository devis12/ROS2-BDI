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

#include "ros2_bdi_skills/communications_client.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define PARAM_AGENT_ID "agent_id"
#define PARAM_AGENT_GROUP_ID "agent_group"
#define PARAM_DEBUG "debug"
  

class BDIActionExecutor : public plansys2::ActionExecutorClient
{
public:
  BDIActionExecutor(const std::string& action_name, const int& working_freq);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
      on_activate(const rclcpp_lifecycle::State & previous_state);

protected:

    /*return arguments of the actions*/
    std::vector<std::string> getArguments() {return get_arguments();}
    /*return current progress state of the action*/
    float getProgress() {return progress_;} 

    /*
        Call to perform at each action activation
    */
    void activation();

    void do_work();

    virtual float advanceWork() = 0;

    /*
      Utility methods to check other agents' desire sets or 
      to send belief request (ADD/DEL) to other agents within the action doWork method implementation
    */
    CheckBeliefResult sendCheckBeliefRequest(const std::string& agentRef, const ros2_bdi_interfaces::msg::Belief& belief);
    UpdBeliefResult sendUpdBeliefRequest(const std::string& agentRef, const ros2_bdi_interfaces::msg::Belief& belief, const UpdOperation& op);
    
    /*
      Utility methods to check other agents' desire sets or 
      send desire request (ADD/DEL) to other agents within the action doWork method implementation
      (flag for monitoring fulfillment is provided for the ADD scenario)
    */
    CheckDesireResult sendCheckDesireRequest(const std::string& agentRef, const ros2_bdi_interfaces::msg::Desire& desire);
    UpdDesireResult sendUpdDesireRequest(const std::string& agentRef, const ros2_bdi_interfaces::msg::Desire& desire, const UpdOperation& op, const bool& monitorFulfill);

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

    std::shared_ptr<CommunicationsClient> comm_client_;

    // progress of the current action execution
    float progress_;
};

#endif  // BDI_ACTION_EXECUTOR_H_
