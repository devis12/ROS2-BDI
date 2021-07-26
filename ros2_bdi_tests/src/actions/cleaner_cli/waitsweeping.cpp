#include <memory>
#include <algorithm>
#include <vector>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define PARAM_AGENT_ID "agent_id"
#define PARAM_SWEEPER_ID "sweeper_id"

using namespace std::chrono_literals;

using std::vector;
using std::string;
using std::bind;
using std::placeholders::_1;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;

class WaitSweeping : public plansys2::ActionExecutorClient
{
public:
  WaitSweeping()
  : plansys2::ActionExecutorClient("waitsweeping", 500ms)
  {
    progress_ = 0.0;
    
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_SWEEPER_ID, "sweeper");
    
    sweeped_ = false;
  }

       rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    sweeped_publisher_ = this->create_publisher<Belief>("add_belief", 10);
    sweeped_publisher_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    sweeped_publisher_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void do_work()
  {
    vector<string> args = get_arguments();
    
    if(progress_ == 0.0)
    { 
      waypoint_ = args[1];
      initSweeperBSetSubscriber();
    }

    if (progress_ < 0.9) {
      progress_ += 0.05;
      send_feedback(progress_, args[0] + " waiting for sweeping in " + args[1] + " running");
    
    } else if(sweeped_){
      notifySweeped();
      finish(true, 1.0, args[0] + " waiting for sweeping in " + args[1] + " completed");

      progress_ = 0.0;
    }
    
    float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
    RCLCPP_INFO(this->get_logger(), 
          "[" + args[0] + " waiting for sweeping in " +  args[1] + "] "+ 
          "progress: %.1f%%", progress_100);
  }

  void initSweeperBSetSubscriber()
  {
    rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
    qos_keep_all.keep_all();
    string sweeper_id = this->get_parameter(PARAM_SWEEPER_ID).as_string();
    sweeper_belief_set_subscriber_ = this->create_subscription<BeliefSet>(
                "/"+sweeper_id+"/belief_set", qos_keep_all,
                bind(&WaitSweeping::sweeperBeliefSetCallback, this, _1));
  }

  void notifySweeped()
  {
    Belief b =  Belief();
    b.pddl_type = Belief().PREDICATE_TYPE;
    b.name = "sweeped";
    b.params = vector<string>({waypoint_});
    sweeped_publisher_->publish(b);
  }

  void sweeperBeliefSetCallback(const BeliefSet::SharedPtr msg)
  {
    BeliefSet sweeperBSet = (*msg);
    for(Belief b : sweeperBSet.value)
      if(b.name == "sweeped" && b.params[0] == waypoint_)
        sweeped_ = true;
  }

  float progress_;
  string waypoint_;
  bool sweeped_;
  
  //listen to belief set of sweeper to know when it's done
  rclcpp::Subscription<BeliefSet>::SharedPtr sweeper_belief_set_subscriber_;
  // belief publisher
  rclcpp_lifecycle::LifecyclePublisher<Belief>::SharedPtr sweeped_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaitSweeping>();
  
  string agent_name = node->get_parameter(PARAM_AGENT_ID).as_string();
  vector<string> specialized_arguments = vector<string>();
  specialized_arguments.push_back(agent_name);

  node->set_parameter(rclcpp::Parameter("action_name", "waitsweeping"));
  node->set_parameter(rclcpp::Parameter("specialized_arguments", specialized_arguments));

  RCLCPP_INFO(node->get_logger(), 
        "\"" + node->get_parameter("action_name").as_string() + 
        "\" action performer for agent: " + node->get_parameter("specialized_arguments").as_string_array()[0]);

  //action node, once created, must pass to inactive state to be ready to execute. 
  // ActionExecutorClient is a managed node (https://design.ros2.org/articles/node_lifecycle.html)
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
