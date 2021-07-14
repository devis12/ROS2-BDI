#include <memory>
#include <algorithm>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define PARAM_AGENT_ID "agent_id"

using namespace std::chrono_literals;

using std::vector;
using std::string;

class DoClean : public plansys2::ActionExecutorClient
{
public:
  DoClean()
  : plansys2::ActionExecutorClient("doclean", 250ms)
  {
    progress_ = 0.0;
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
  }

private:
  void do_work()
  {
    vector<string> args = get_arguments();
    if (progress_ < 1.0) {
      progress_ += 0.0625;
      send_feedback(progress_, args[0] + "cleaning " + args[1] + " running");
    } else {
      finish(true, 1.0, args[0] + " cleaning " + args[1] + " completed");

      progress_ = 0.0;
    }

    float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
    RCLCPP_DEBUG(this->get_logger(), 
        "[" + args[0] + " cleaning " +  args[1] + "] "+ 
        "progress: %.1f%%", progress_100);
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DoClean>();
  
  string agent_name = node->get_parameter(PARAM_AGENT_ID).as_string();
  vector<string> specialized_arguments = vector<string>();
  specialized_arguments.push_back(agent_name);

  node->set_parameter(rclcpp::Parameter("action_name", "doclean"));
  node->set_parameter(rclcpp::Parameter("specialized_arguments", specialized_arguments));

  RCLCPP_DEBUG(node->get_logger(), 
        "\"" + node->get_parameter("action_name").as_string() + 
        "\" action performer for agent: " + node->get_parameter("specialized_arguments").as_string_array()[0]);

  //action node, once created, must pass to inactive state to be ready to execute. 
  // ActionExecutorClient is a managed node (https://design.ros2.org/articles/node_lifecycle.html)
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
