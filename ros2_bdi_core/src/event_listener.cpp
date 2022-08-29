#include <vector>
#include <map>

// header file for Event listener node
#include "ros2_bdi_core/event_listener.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Event Listener node
#include "ros2_bdi_core/params/event_listener_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node
#include "ros2_bdi_core/params/belief_manager_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Scheduler node
#include "ros2_bdi_core/params/scheduler_params.hpp"

#include "ros2_bdi_utils/BDIYAMLParser.hpp"

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::LifecycleStatus;

using BDIManaged::ManagedBelief;
using BDIManaged::ManagedReactiveRule;
using std::string;
using std::vector;
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
    rclcpp::QoS qos_reliable = rclcpp::QoS(10);
    qos_reliable.reliable();


    // initializing domain expert
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();

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
                LIFECYCLE_STATUS_TOPIC, qos_reliable,
                bind(&EventListener::callbackLifecycleStatus, this, _1));

    //Receive belief set update notification to keep the event listener belief set mirror up to date
    belief_set_subscription_ = this->create_subscription<BeliefSet>(
                BELIEF_SET_TOPIC, qos_reliable,
                bind(&EventListener::updBeliefSetCallback, this, _1));

    // add/del belief publishers init.
    add_belief_publisher_ = this->create_publisher<Belief>(ADD_BELIEF_TOPIC, qos_reliable);
    del_belief_publisher_ = this->create_publisher<Belief>(DEL_BELIEF_TOPIC, qos_reliable);

    // add/del desire publishers init.
    add_desire_publisher_ = this->create_publisher<Desire>(ADD_DESIRE_TOPIC, qos_reliable);
    del_desire_publisher_ = this->create_publisher<Desire>(DEL_DESIRE_TOPIC, qos_reliable);

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
        rules = BDIYAMLParser::extractMGReactiveRules(init_reactive_rules_filepath, domain_expert_);//TODO test

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


/*Apply satisfying rules starting from extracted possible assignments for the placeholders*/
void EventListener::apply_satisfying_rules(const ManagedReactiveRule& reactive_rule, const map<string, vector<ManagedBelief>>& assignments, const set<ManagedBelief>& belief_set)
{
    vector<int> current_choices = vector<int>(assignments.size(),0); // init current_choices as {0,0,0,...} with length == num of placeholders
    
    vector<int> limit_choices = vector<int>(assignments.size(),0); // init limit_choices with max value per each digit, e.g. {3, 4, 6}
    
    bool reached_end = false;
    int i = 0;
    for(auto it = assignments.begin(); !reached_end && it != assignments.end(); ++it)
    {
        limit_choices[i] = it->second.size();
        if(limit_choices[i] > 0)//there seems to be at least a combination to evaluate
            reached_end = false;
        else
            reached_end = true;//stop immediately, a placeholder has no possible actual replacement with an instance
        i++;
    }

    while(!reached_end) // keep iterating while there is a combination to evaluate
    {
        // use current choice and apply them to replace placeholder in reactive_rule with actual values from possible assignments
        map<string, string> actual_assignments;
        i = 0;
        for(auto it = assignments.begin(); it != assignments.end(); ++it)
        {
            actual_assignments[it->first] = it->second[current_choices[i]].getName();
            i++;
        }

        ManagedReactiveRule reactive_rule_subs = ManagedReactiveRule::applySubstitution(reactive_rule, actual_assignments);
        if(reactive_rule_subs.getMGCondition().isSatisfied(belief_set_))
            apply_rule(reactive_rule_subs);

        // try to update current choice
        bool upd = false;
        // increment one
        for(int i = 0; i<current_choices.size(); i++)
        {
            if(current_choices[i] < limit_choices[i] - 1)
            {
                current_choices[i]++;
                if(i>0)
                {
                    int j = 0;
                    while(j<i)
                    {
                        current_choices[j] = 0;
                        j++;
                    }
                }
                upd = true;
                i = current_choices.size(); //exit immediately
            }
        }
        reached_end = !upd;//if upd made -> not reached end yet
    }
}


/*Iterate over the rules and check if any of them applies, if yes enforces it*/
void EventListener::check_if_any_rule_apply()
{
    for(auto reactive_rule : reactive_rules_)
    {  
        map <string, vector<ManagedBelief>> assignments;//contains variable assigments
        if(reactive_rule.getMGCondition().containsPlaceholders())
        {
            assignments = reactive_rule.getMGCondition().extractAssignmentsMap(belief_set_);
            apply_satisfying_rules(reactive_rule, assignments, belief_set_);
        }

        else if(reactive_rule.getMGCondition().isSatisfied(belief_set_))//rule with no placeholders sat -> apply rune as is
        {
            //rule is satisfied! apply effects
            apply_rule(reactive_rule);
        }
    }
}

/*Apply reactive rule, by publishing to the right topic belief/desire set updates as defined in reactive_rule*/
void EventListener::apply_rule(const BDIManaged::ManagedReactiveRule& reactive_rule)
{
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
