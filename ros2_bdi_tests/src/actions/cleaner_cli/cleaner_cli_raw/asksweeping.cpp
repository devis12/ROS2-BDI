#include <memory>
#include <algorithm>
#include <vector>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define PARAM_AGENT_ID "agent_id"
#define PARAM_SWEEPER_ID "sweeper_id"

using namespace std::chrono_literals;

using std::vector;
using std::string;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;

class AskSweeping : public plansys2::ActionExecutorClient
{
public:
  AskSweeping()
  : plansys2::ActionExecutorClient("asksweeping", 250ms)
  {
    progress_ = 0.0;
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_SWEEPER_ID, "sweeper");
  }

     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    string sweeper_id = this->get_parameter(PARAM_SWEEPER_ID).as_string();
    sweeper_desire_publisher_ = this->create_publisher<Desire>("/" + sweeper_id + "/add_desire", 10);
    progress_ = 0.0f;
    sweeper_desire_publisher_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    sweeper_desire_publisher_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void do_work()
  {
    vector<string> args = get_arguments();
    progress_ += 0.5;
    
    if (progress_ < 1.0) {
      send_feedback(progress_, args[0] + " asksweeping " + args[1] + " running");
      askToSweep(args[1]);
    } else {
      finish(true, 1.0, args[0] + " asksweeping " + args[1] + " completed");
    }

    float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
    RCLCPP_INFO(this->get_logger(), 
        "[" + args[0] + " asksweeping " +  args[1] + "] "+ 
        "progress: %.1f%%", progress_100);
  }

  /*
    Inject desire into cleaner desire set
  */
  void askToSweep(const string& waypoint)
  {
      vector<Belief> target;
      Belief b =  Belief();
      b.pddl_type = Belief().PREDICATE_TYPE;
      b.name = "sweeped";
      b.params = vector<string>({waypoint});
      target.push_back(b);

      Desire desire = Desire();
      desire.name = "sweep_" + waypoint;
      desire.deadline = 16.0;
      desire.priority = 1.0;
      desire.value = target;
      
      sweeper_desire_publisher_->publish(desire);
  }

  float progress_;
  // desire publisher
  rclcpp_lifecycle::LifecyclePublisher<Desire>::SharedPtr sweeper_desire_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AskSweeping>();
  
  string agent_name = node->get_parameter(PARAM_AGENT_ID).as_string();
  vector<string> specialized_arguments = vector<string>();
  specialized_arguments.push_back(agent_name);

  node->set_parameter(rclcpp::Parameter("action_name", "asksweeping"));
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
