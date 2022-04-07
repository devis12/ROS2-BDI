#ifndef EVENT_LISTENER_H_
#define EVENT_LISTENER_H_

#include <string>
#include <set>
#include <utility>
#include <memory>
#include <mutex>  

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedReactiveRule.hpp"

#include "ros2_bdi_utils/BDIFilter.hpp"

#include "rclcpp/rclcpp.hpp"

/*
    Checks at every belief set update for conditions to be satisfied,
    if any of them is sat, then apply the corresponding rules

    If no rules are defined at start, the node automatically shuts down 
*/
class EventListener : public rclcpp::Node
{
    public:

        /* Constructor method */
        EventListener();

        /*
            Init to call at the start, after construction method, to get the node actually started
            initializing publishers to add/del belief/desire topics of the agent
            and subscriber to the belief_set topic.
            Policies to check are retrieved and load from a static file and cannot be modified at run time (yet)

            Returns false if no reactive rule were to be found
        */
        bool init();

    private:

        void updBeliefSetCallback(const ros2_bdi_interfaces::msg::BeliefSet::SharedPtr msg)
        {
            belief_set_ = BDIFilter::extractMGBeliefs(msg->value);
            check_if_any_rule_apply();
        }

        // recover from expected file the rules to be applied
        std::set<BDIManaged::ManagedReactiveRule> init_reactive_rules();

        /*Iterate over the rules and check if any of them applies, if yes enforces it*/
        void check_if_any_rule_apply();

        // internal state of the node
        // StateType state_; //NOT used in this node (it just reacts to belief set update notification)
               
        // agent id that defines the namespace in which the node operates
        std::string agent_id_;

        //policy rules set
        std::set<BDIManaged::ManagedReactiveRule> reactive_rules_;

        std::set<BDIManaged::ManagedBelief> belief_set_;

        // belief set publishers
        rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr add_belief_publisher_;//add belief topic pub
        rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr del_belief_publisher_;//del belief topic pub

        // desire publishers
        rclcpp::Publisher<ros2_bdi_interfaces::msg::Desire>::SharedPtr add_desire_publisher_;//add desire topic pub
        rclcpp::Publisher<ros2_bdi_interfaces::msg::Desire>::SharedPtr del_desire_publisher_;//del desire topic pub

        rclcpp::Subscription<ros2_bdi_interfaces::msg::BeliefSet>::SharedPtr belief_set_subscription_;//belief set subscription

}; //BeliefManager class prototype

#endif //EVENT_LISTENER_H_
