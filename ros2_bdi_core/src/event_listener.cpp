#include <string>

// header file for Event listener node
#include "ros2_bdi_core/event_listener.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Event Listener node
#include "ros2_bdi_core/params/event_listener_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node
#include "ros2_bdi_core/params/belief_manager_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Scheduler node
#include "ros2_bdi_core/params/scheduler_params.hpp"

#include "ros2_bdi_utils/BDIYAMLParser.hpp"

using ros2_bdi_interfaces::msg::LifecycleStatus;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::Desire;

using BDIManaged::ManagedReactiveRule;
using std::string;
using std::set;
using std::map;
using std::bind;
using std::placeholders::_1;

EventListener::EventListener()
    : rclcpp::Node(EVENT_LISTENER_NODE_NAME)//, state_(STARTING)
{
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);

    this->declare_parameter(PARAM_PLANNING_MODE, PLANNING_MODE_OFFLINE);

    sel_planning_mode_ = this->get_parameter(PARAM_PLANNING_MODE).as_string() == PLANNING_MODE_OFFLINE? OFFLINE : ONLINE;
    this->undeclare_parameter(PARAM_PLANNING_MODE);
}

bool EventListener::init()
{ 
    //agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();
    rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
    qos_keep_all.keep_all();

    // init rules set
    reactive_rules_ = init_reactive_rules();
    if(reactive_rules_.size() == 0)
    {   
        rclcpp::shutdown();
        return false;
    }

    //lifecycle status init
    auto lifecycle_status = LifecycleStatus{};
    lifecycle_status_ = map<string, uint8_t>();
    lifecycle_status_[BELIEF_MANAGER_NODE_NAME] = lifecycle_status.BOOTING;
    lifecycle_status_[SCHEDULER_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[PLAN_DIRECTOR_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[PSYS_MONITOR_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[EVENT_LISTENER_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[MA_REQUEST_HANDLER_NODE_NAME] = lifecycle_status.UNKNOWN;

    // init step_counter
    step_counter_ = 0;

    //Lifecycle status publisher
    lifecycle_status_publisher_ = this->create_publisher<LifecycleStatus>(LIFECYCLE_STATUS_TOPIC, 10);

    //Lifecycle status subscriber
    lifecycle_status_subscriber_ = this->create_subscription<LifecycleStatus>(
                LIFECYCLE_STATUS_TOPIC, qos_keep_all,
                bind(&EventListener::callbackLifecycleStatus, this, _1));

    //Receive belief set update notification to keep the event listener belief set mirror up to date
    belief_set_subscription_ = this->create_subscription<BeliefSet>(
                BELIEF_SET_TOPIC, qos_keep_all,
                bind(&EventListener::updBeliefSetCallback, this, _1));

    // add/del belief publishers init.
    add_belief_publisher_ = this->create_publisher<Belief>(ADD_BELIEF_TOPIC, qos_keep_all);
    del_belief_publisher_ = this->create_publisher<Belief>(DEL_BELIEF_TOPIC, qos_keep_all);

    // add/del desire publishers init.
    add_desire_publisher_ = this->create_publisher<Desire>(ADD_DESIRE_TOPIC, qos_keep_all);
    del_desire_publisher_ = this->create_publisher<Desire>(DEL_DESIRE_TOPIC, qos_keep_all);

    return true;
}

/*Build updated LifecycleStatus msg*/
LifecycleStatus EventListener::getLifecycleStatus()
{
    LifecycleStatus lifecycle_status = LifecycleStatus{};
    lifecycle_status.node_name = EVENT_LISTENER_NODE_NAME;
    lifecycle_status.status = state_ == CHECKING? lifecycle_status.RUNNING : lifecycle_status.BOOTING;
    return lifecycle_status;
}

/*
    Expect to find yaml file to init the belief set in "/tmp/{agent_id}/init_bset.yaml"
*/
set<ManagedReactiveRule> EventListener::init_reactive_rules()
{
    string init_reactive_rules_filepath = "/tmp/"+this->get_parameter(PARAM_AGENT_ID).as_string()+"/"+INIT_REACTIVE_RULES_FILENAME;
    set<ManagedReactiveRule> rules;

    try{
        rules = BDIYAMLParser::extractMGReactiveRules(init_reactive_rules_filepath);

        if(this->get_parameter(PARAM_DEBUG).as_bool())
            RCLCPP_INFO(this->get_logger(), "Reactive rules initialization performed through " + init_reactive_rules_filepath);
    
    }catch(const YAML::BadFile& bfile){
        RCLCPP_ERROR(this->get_logger(), "Bad File: Reactive rules initialization failed because init. file " + init_reactive_rules_filepath + " hasn't been found");
    }catch(const YAML::ParserException& bpars){
        RCLCPP_ERROR(this->get_logger(), "YAML Parser Exception: Reactive rules initialization failed because init. file " + init_reactive_rules_filepath + " doesn't present a valid YAML format");
    }catch(const YAML::BadConversion& bconvfile){
        RCLCPP_ERROR(this->get_logger(), "Bad Conversion: Reactive rules initialization failed because init. file " + init_reactive_rules_filepath + " doesn't present a valid reactive rules array");
    }catch(const YAML::InvalidNode& invalid_node){
        RCLCPP_ERROR(this->get_logger(), "Invalid Node: Reactive rules initialization failed because init. file " + init_reactive_rules_filepath + " doesn't present a valid reactive rules array");
    }

    return rules;
}


/*Iterate over the rules and check if any of them applies, if yes enforces it*/
void EventListener::check_if_any_rule_apply()
{
    for(auto reactive_rule : reactive_rules_)
    {
        if(reactive_rule.getMGCondition().isSatisfied(belief_set_))
        {
            //rule is satisfied! apply effects

            //Belief set updates
            for(auto bset_upd : reactive_rule.getBeliefRules())
            {
                auto bel_to_string = bset_upd.second.getName() + " " + bset_upd.second.getParamsJoined() 
                    + " value = " + std::to_string (bset_upd.second.getValue()) ;
                if(bset_upd.first == ReactiveOp::ADD)
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Adding belief " + bel_to_string);

                    add_belief_publisher_->publish(bset_upd.second.toBelief());//add belief to bset 
                }
                else
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Deleting belief " + bel_to_string);

                    del_belief_publisher_->publish(bset_upd.second.toBelief());//del belief to bset
                }
            }

            //Desire set updates
            for(auto dset_upd : reactive_rule.getDesireRules())
            {
                if(dset_upd.first == ReactiveOp::ADD)
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Adding desire " + dset_upd.second.getName());

                    add_desire_publisher_->publish(dset_upd.second.toDesire());//add desire to dset 
                }
                else
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Deleting desire " + dset_upd.second.getName());

                    del_desire_publisher_->publish(dset_upd.second.toDesire());//del desire to dset
                }
            }
        }
    }
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EventListener>();
  bool psys2_booted = node->wait_psys2_boot(std::chrono::seconds(8));//Wait max 8 seconds for plansys2 to boot
  
  if(psys2_booted)
  {
    if(node->init())//if False, no proper reactive rules defined -> hence no point in having the node booted up 
        rclcpp::spin(node);
    else
        std::cout << "Event Listener has provided with any rule to implement Belief Revision Function or Desire Generation Function, thus will be directly terminated" << std::endl;
  }
  else
  {
    std::cerr << "PlanSys2 failed to boot: node will not spin and process will terminate" << std::endl;
  }
    
  rclcpp::shutdown();

  return 0;
}
