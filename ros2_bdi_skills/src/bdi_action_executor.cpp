// header file for bdi action executor abstract class
#include "ros2_bdi_skills/bdi_action_executor.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node (for belief set topic)
#include "ros2_bdi_core/params/belief_manager_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Scheduler node (for belief set topic)
#include "ros2_bdi_core/params/scheduler_params.hpp"

using std::string;
using std::vector;
using std::set;
using std::map;
using std::tuple;
using std::pair;
using std::shared_ptr;
using std::thread;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::mutex;
using std::optional;

using plansys2::ProblemExpertClient;
using plansys2::ExecutorClient;

using ros2_bdi_interfaces::msg::Belief;            
using ros2_bdi_interfaces::msg::BeliefSet;            
using ros2_bdi_interfaces::msg::Desire;  
using ros2_bdi_interfaces::srv::CheckBelief;  
using ros2_bdi_interfaces::srv::UpdBeliefSet;  
using ros2_bdi_interfaces::srv::CheckDesire;  
using ros2_bdi_interfaces::srv::UpdDesireSet;  

using javaff_interfaces::msg::ActionExecutionStatus;
using javaff_interfaces::msg::ExecutionStatus;

using BDIManaged::ManagedBelief;
using BDIManaged::ManagedDesire;

using BDICommunications::UpdOperation;
using BDICommunications::CheckBeliefResult;
using BDICommunications::CheckDesireResult;
using BDICommunications::UpdBeliefResult;
using BDICommunications::UpdDesireResult;
using BDICommunications::CommunicationsClient;


/*
  Constructor for every action executor node, 
    @action_name should match the one within the pddl domain definition
    @working_freq represents the frequency at which the doWork method is called (expressed in Hz)
*/
BDIActionExecutor::BDIActionExecutor(const string action_name, const int working_freq, const bool agent_id_as_specialized_arg) : 
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
      if(agent_id_as_specialized_arg)
        specialized_arguments.push_back(agent_id_);

      // lifecycle publisher to communicate exec status to online planner
      exec_status_to_planner_publisher_ = this->create_publisher<ExecutionStatus>("/"+agent_id_+"/"+JAVAFF_EXEC_STATUS_TOPIC, 
                rclcpp::QoS(1).reliable());

      this->set_parameter(rclcpp::Parameter("action_name", action_name_));
      this->set_parameter(rclcpp::Parameter("specialized_arguments", specialized_arguments));

      // action node, once created, must pass to inactive state to be ready to execute. 
      // ActionExecutorClient is a managed node (https://design.ros2.org/articles/node_lifecycle.html)
      this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  }

/*
  Method called when node is triggered by the Executor node of PlanSys2
  Progress for the action sets to 0.0f
*/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  BDIActionExecutor::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    //problem expert client to communicate with problem expert node of plansys2
    problem_expert_ = std::make_shared<ProblemExpertClient>();

    //executor client to communicate with executor node of plansys2
    executor_client_ = std::make_shared<ExecutorClient>();

    progress_ = 0.0f;
    if(this->get_parameter(PARAM_DEBUG).as_bool())
      RCLCPP_INFO(this->get_logger(), "Action executor controller for \"" + action_name_ + "\" ready for execution");
    
    exec_status_to_planner_publisher_->on_activate();
    
    communicateExecStatus(ActionExecutionStatus().RUNNING);
    
    return ActionExecutorClient::on_activate(previous_state);
  }

  void BDIActionExecutor::communicateExecStatus(const uint8_t& status)
  {
    //notify javaff about executing status (action is about to start)
    ExecutionStatus exec_status_msg = ExecutionStatus();
    exec_status_msg.executing_plan_index = get_executing_plan_index();
    exec_status_msg.pddl_problem = problem_expert_->getProblem();
    
    //Put just info wrt this action execution and not others (NOT needed)
    plansys2_msgs::action::ExecutePlan::Feedback actionsFeedback = executor_client_->getFeedBack(true);
    exec_status_msg.sim_to_goal = actionsFeedback.early_abort_accepted? exec_status_msg.NO_SIM_TO_GOAL : exec_status_msg.SIM_TO_GOAL;

    for(auto actionExecInfo: actionsFeedback.action_execution_status){
      ActionExecutionStatus  action_exec_status_msg = ActionExecutionStatus();
      std::size_t par1Pos = actionExecInfo.action_full_name.find("(");
      std::size_t par2Pos = actionExecInfo.action_full_name.find(")");
      std::size_t colPos = actionExecInfo.action_full_name.find(":");
      std::string actionFullNameNoTime = actionExecInfo.action_full_name.substr(par1Pos, (par2Pos+1)-par1Pos);
      action_exec_status_msg.executing_action = actionFullNameNoTime;
      action_exec_status_msg.planned_start_time = std::stof(actionExecInfo.action_full_name.substr(colPos+1))/1000.0f;
      
      if(actionExecInfo.action_full_name==getFullActionName())
      {
        action_exec_status_msg.status = status;
        if(status == action_exec_status_msg.SUCCESS)// check whether at end effects already applied in this case(they shouldn't, hence mark it as hybrid state RUN_SUC)
          action_exec_status_msg.status = actionExecInfo.at_end_applied? status : action_exec_status_msg.RUN_SUC;
      }
      else
      {
        switch(actionExecInfo.status)
        {
          case actionExecInfo.NOT_EXECUTED:
            action_exec_status_msg.status = actionExecInfo.at_start_applied? action_exec_status_msg.RUNNING : action_exec_status_msg.WAITING;
            break;
          case actionExecInfo.EXECUTING:
            action_exec_status_msg.status = action_exec_status_msg.RUNNING;
            break;
          case actionExecInfo.FAILED:
          case actionExecInfo.CANCELLED:
            action_exec_status_msg.status = action_exec_status_msg.FAILURE;
            break;
          case actionExecInfo.SUCCEEDED:
            action_exec_status_msg.status = actionExecInfo.at_end_applied? action_exec_status_msg.SUCCESS : action_exec_status_msg.RUN_SUC;
            break;
          default:
            action_exec_status_msg.status = action_exec_status_msg.FAILURE;
            break;
        }
      }
      
      exec_status_msg.executing_actions.push_back(action_exec_status_msg);
    }
    
    exec_status_to_planner_publisher_->publish(exec_status_msg);
  }

/*
  Method called when node is deactivate by the Executor node of PlanSys2
  cleanup of monitored_desires, subscription, belief set if needed
*/
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    BDIActionExecutor::on_deactivate(const rclcpp_lifecycle::State & previous_state) 
  {
    if(problem_expert_.use_count() > 0)
      problem_expert_.reset();

    if(executor_client_.use_count() > 0)
      executor_client_.reset();
    
    monitored_bsets_.clear();
    for(auto monitor_desire : monitored_desires_)
      std::get<2>(monitor_desire).reset();//should allow to cancel subscription to topic (https://answers.ros.org/question/354792/rclcpp-how-to-unsubscribe-from-a-topic/)
    monitored_desires_.clear();

    exec_status_to_planner_publisher_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

/*
  Method regularly called at the frequency specified in the constructor to handle the advancement of the action,
  wrapper method for the domain specific logics written within advanceWork by the user. Its purpose is to hide
  calls that has to be made to PlanSys2, to avoid the user the burden to check a second docs.
*/
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
CheckBeliefResult BDIActionExecutor::sendCheckBeliefRequest(const string& agent_ref, const Belief& belief)
{
  return comm_client_->checkBeliefRequest(agent_ref, agent_group_, belief);
}

UpdBeliefResult BDIActionExecutor::sendUpdBeliefRequest(const string& agent_ref, const Belief& belief, const UpdOperation& op)
{
  return comm_client_->updBeliefRequest(agent_ref, agent_group_, belief, op);
}


/*
  Utility methods to check other agents' desire sets or 
  send desire request (ADD/DEL) to other agents within the action doWork method implementation
  (flag for monitoring fulfillment is provided for the ADD scenario)
*/

CheckDesireResult BDIActionExecutor::sendCheckDesireRequest(const string& agent_ref, const Desire& desire)
{
  return comm_client_->checkDesireRequest(agent_ref, agent_group_, desire);
}

UpdDesireResult BDIActionExecutor::sendUpdDesireRequest(const string& agent_ref, const Desire& desire, const UpdOperation& op, const bool& monitor_fulfill)
{
  auto res = comm_client_->updDesireRequest(agent_ref, agent_group_, desire, op);
  if(res.accepted && res.performed && monitor_fulfill)
    monitor(agent_ref, desire);
  return res;
}

/*
  if no monitored desire, just return false
  otherwise check if it is fulfilled in the respective monitored belief set
*/
bool BDIActionExecutor::isMonitoredDesireFulfilled(const std::string& agent_ref, const ros2_bdi_interfaces::msg::Desire& desire)
{
  for(auto monitor_desire : monitored_desires_)
  {
    if(std::get<0>(monitor_desire) == agent_ref && std::get<1>(monitor_desire) == ManagedDesire{desire})
      return monitored_bsets_.find(agent_ref) != monitored_bsets_.end() &&
        (ManagedDesire{desire}).isFulfilled(monitored_bsets_.find(agent_ref)->second);
  }   
  return false;
}

/*
  Monitor belief set update of an agent 
*/
void BDIActionExecutor::monitor(const string& agent_ref, const Desire& desire)
{
  rclcpp::QoS qos_reliable = rclcpp::QoS(10);
  qos_reliable.reliable();

  rclcpp::Subscription<ros2_bdi_interfaces::msg::BeliefSet>::SharedPtr agent_belief_set_subscriber = this->create_subscription<BeliefSet>(
            "/"+agent_ref+"/"+BELIEF_SET_TOPIC, qos_reliable,
            bind(&BDIActionExecutor::agentBeliefSetCallback, this, _1));

  monitored_desires_.push_back(std::make_tuple(agent_ref, ManagedDesire{desire}, agent_belief_set_subscriber));
}

/*
  update the current monitored belief set 
*/
void BDIActionExecutor::agentBeliefSetCallback(const BeliefSet::SharedPtr msg)
{
  map<string, set<ManagedBelief>>::iterator it = monitored_bsets_.find(msg->agent_id);
  if(it != monitored_bsets_.end())
    it->second = BDIFilter::extractMGBeliefs(msg->value);
  else
    monitored_bsets_.insert(monitored_bsets_.begin(), pair<string, set<ManagedBelief>>(msg->agent_id, BDIFilter::extractMGBeliefs(msg->value)));
}
