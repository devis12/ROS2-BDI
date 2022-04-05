#include <memory>
#include <algorithm>
#include <vector>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define PARAM_AGENT_ID "agent_id"
#define PARAM_AGENT_GROUP_ID "agent_group"

using namespace std::chrono_literals;

using std::vector;
using std::string;
using std::bind;
using std::placeholders::_1;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::Desire;

class AskSweeping : public plansys2::ActionExecutorClient
{
public:
  AskSweeping()
  : plansys2::ActionExecutorClient("asksweeping", 250ms)
  {
    progress_ = 0.0;
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_AGENT_GROUP_ID, "agent0_group");
  }

     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    vector<string> args = get_arguments();
    string sweeper_id = args[1];
    waypoint_ = args[2];
    initSweeperBSetSubscriber(sweeper_id);
    
    cleaner_add_bel_publisher_ = this->create_publisher<Belief>("add_belief", 10);
    cleaner_add_bel_publisher_->on_activate();
    
    sweeper_desire_publisher_ = this->create_publisher<Desire>("/" + sweeper_id + "/add_desire", 10);
    sweeper_desire_publisher_->on_activate();

    progress_ = 0.0f;
    swept_ = false;

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    cleaner_add_bel_publisher_->on_deactivate();
    sweeper_desire_publisher_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:

  //subscribe to sweeper belief set
  void initSweeperBSetSubscriber(const  string& sweeper_id)
  {
    rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
    qos_keep_all.keep_all();
    sweeper_belief_set_subscriber_ = this->create_subscription<BeliefSet>(
                "/"+sweeper_id+"/belief_set", qos_keep_all,
                bind(&AskSweeping::sweeperBeliefSetCallback, this, _1));
  }


  //receive update of sweeper belief set
  void sweeperBeliefSetCallback(const BeliefSet::SharedPtr msg)
  {
    BeliefSet sweeperBSet = (*msg);
    for(Belief b : sweeperBSet.value)
      if(b.name == "swept" && b.params[0] == waypoint_)
        swept_ = true;
    /*
    if(!swept_)
      std::cout << waypoint_ << " still not swept by sweeper" << std::endl;
    else
      std::cout << waypoint_ << " finally swept by sweeper!" << std::endl;*/
  }

  // notify waypoint has been swept to cleaner itself
  void notifySwept()
  {
    Belief b =  Belief();
    b.pddl_type = Belief().PREDICATE_TYPE;
    b.name = "swept";
    b.params = vector<string>({waypoint_});
    cleaner_add_bel_publisher_->publish(b);
  }

  void do_work()
  {
    vector<string> args = get_arguments();

    if(progress_ == 0.0f)
        askToSweep(waypoint_);//first do_work call -> inject desire into sweeper

    //advance progress to 100% if swept. otherwise advance slowly iff you do not reach or go over 100%
    progress_ += (swept_)? 1.0f : ((progress_ + 0.0625f) >= 1.0f)? 0.0f : 0.0625f;
    
    if (progress_ < 1.0) {
      send_feedback(progress_, args[0] + " asksweeping to " + args[1] + " for wp " + args[2] + " running");
    } else {
      notifySwept();
      finish(true, 1.0, args[0] + " asksweeping to " + args[1] + " for wp " + args[2] + " completed");
    }

    float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
    RCLCPP_INFO(this->get_logger(), 
        "[" + args[0] + " asksweeping to " + args[1] + " for wp " + args[2] + "] "+ 
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
      b.name = "swept";
      b.params = vector<string>({waypoint});
      target.push_back(b);

      Desire desire = Desire();
      desire.name = "sweep_" + waypoint;
      desire.deadline = 16.0;
      desire.priority = 0.6;
      desire.value = target;
      
      sweeper_desire_publisher_->publish(desire);
  }

  float progress_;
  // desire publisher
  rclcpp_lifecycle::LifecyclePublisher<Desire>::SharedPtr sweeper_desire_publisher_;

  // waypoint has been swept
  bool swept_;
  // name of the waypoint you're waiting to be swept
  string waypoint_;
  //listen to belief set of sweeper to know when it's done
  rclcpp::Subscription<BeliefSet>::SharedPtr sweeper_belief_set_subscriber_;
  // add belief publisher
  rclcpp_lifecycle::LifecyclePublisher<Belief>::SharedPtr cleaner_add_bel_publisher_;
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
