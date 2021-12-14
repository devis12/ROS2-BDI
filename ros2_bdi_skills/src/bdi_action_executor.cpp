#include "ros2_bdi_skills/bdi_action_executor.hpp"

using std::string;
using std::vector;
using std::set;
using std::shared_ptr;
using std::thread;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::mutex;
using std::optional;

using ros2_bdi_interfaces::msg::Belief;            
using ros2_bdi_interfaces::msg::BeliefSet;            
using ros2_bdi_interfaces::msg::Desire;  
using ros2_bdi_interfaces::srv::CheckBelief;  
using ros2_bdi_interfaces::srv::UpdBeliefSet;  
using ros2_bdi_interfaces::srv::CheckDesire;  
using ros2_bdi_interfaces::srv::UpdDesireSet;  

using BDIManaged::ManagedBelief;
using BDIManaged::ManagedDesire;


BDIActionExecutor::BDIActionExecutor(const string& action_name, const int& working_freq) : 
  plansys2::ActionExecutorClient(action_name, milliseconds((int) (1000/working_freq)))
  {
      this->declare_parameter(PARAM_AGENT_ID, "agent0");
      this->declare_parameter(PARAM_AGENT_GROUP_ID, "agent0_group");
      this->declare_parameter(PARAM_DEBUG, true);

      // store action name
      action_name_ = action_name;
      
      // agent's namespace
      agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

      // agent's group name
      agent_group_ = this->get_parameter(PARAM_AGENT_GROUP_ID).as_string();

      comm_client_ = std::make_shared<CommunicationsClient>();

      // set agent id as specialized arguments
      vector<string> specialized_arguments = vector<string>();
      specialized_arguments.push_back(agent_id_);

      this->set_parameter(rclcpp::Parameter("action_name", action_name_));
      this->set_parameter(rclcpp::Parameter("specialized_arguments", specialized_arguments));

      // action node, once created, must pass to inactive state to be ready to execute. 
    // ActionExecutorClient is a managed node (https://design.ros2.org/articles/node_lifecycle.html)
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  BDIActionExecutor::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    activation();
    std::cout << "\n\n!!! (" << this->get_parameter("action_name").as_string() << ") activated!\n\n" << std::endl;
    return ActionExecutorClient::on_activate(previous_state);
  }

  /*
      Call to perform at each action activation
  */
  void BDIActionExecutor::activation()
  { 
      // status set to initial status 0.0%
      progress_ = 0.0f;

      RCLCPP_INFO(this->get_logger(), "Action executor controller for \"" + action_name_ + "\" ready for execution");

  }

  void BDIActionExecutor::do_work()
  {
    action_params_ = get_arguments();//get arguments of action execution

    progress_ += advanceWork();
    if(progress_ >= 1.0)
      execSuccess();//send feedback of complete execution to plansys2 executor (node deactivate again)

    else
    {
      float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
      string progress_100S = std::to_string(progress_100);
      progress_100S = progress_100S.substr(0, progress_100S.find(".", 0)+2);//just save one decimal

      string params_string = "";
      for(int i = 0; i < action_params_.size(); i++)
        params_string += " " + action_params_[i];
      
      //feedback string to be sent to plansys2 executor along with the progress_status (logged if debug=TRUE)
      string feedback_string = string("[" + action_name_ +  params_string
            + "] "+ progress_100S + "%%");
      send_feedback(progress_, feedback_string);//send feedback to plansys2 executor

      if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_INFO(this->get_logger(), feedback_string);
    }
  }

  /*
      Utility methods to check other agents' desire sets or 
      to send belief request (ADD/DEL) to other agents within the action doWork method implementation
  */
  CheckBeliefResult BDIActionExecutor::sendCheckBeliefRequest(const string& agentRef, const Belief& belief)
  {
    return comm_client_->checkBeliefRequest(agentRef, agent_group_, belief);
  }

  UpdBeliefResult BDIActionExecutor::sendUpdBeliefRequest(const string& agentRef, const Belief& belief, const UpdOperation& op)
  {
    return comm_client_->updBeliefRequest(agentRef, agent_group_, belief, op);
  }


  /*
    Utility methods to check other agents' desire sets or 
    send desire request (ADD/DEL) to other agents within the action doWork method implementation
    (flag for monitoring fulfillment is provided for the ADD scenario)
  */

  CheckDesireResult BDIActionExecutor::sendCheckDesireRequest(const string& agentRef, const Desire& desire)
  {
    return comm_client_->checkDesireRequest(agentRef, agent_group_, desire);
  }

  UpdDesireResult BDIActionExecutor::sendUpdDesireRequest(const string& agentRef, const Desire& desire, const UpdOperation& op, const bool& monitorFulfill)
  {
    auto res = comm_client_->updDesireRequest(agentRef, agent_group_, desire, op);
    if(res.accepted && res.performed && monitorFulfill)
      monitor(agentRef, desire);
    return res;
  }

  /*
    if no monitored desire, just return false
    otherwise check if it is fulfilled in the respective monitored belief set
  */
  bool BDIActionExecutor::isMonitoredDesireSatisfied()
  {
    if(monitored_desire_ == ManagedDesire{})
      return false;
    else
      return monitored_desire_.isFulfilled(monitored_beliefset_);
  }

  /*
    Monitor belief set update of an agent 
  */
  void BDIActionExecutor::monitor(const string& agentRef, const Desire& desire)
  {
    rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
    qos_keep_all.keep_all();

    agent_belief_set_subscriber_ = this->create_subscription<BeliefSet>(
              "/"+agentRef+"/belief_set", qos_keep_all,
              bind(&BDIActionExecutor::agentBeliefSetCallback, this, _1));
    
    monitored_desire_ = ManagedDesire{desire};
  }

  /*
    update the current monitored belief set 
  */
  void BDIActionExecutor::agentBeliefSetCallback(const BeliefSet::SharedPtr msg)
  {
      monitored_beliefset_ = BDIFilter::extractMGBeliefs(msg->value);
  }
