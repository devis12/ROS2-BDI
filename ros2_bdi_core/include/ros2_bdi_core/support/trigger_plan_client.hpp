#ifndef TRIGGER_PLAN_CLIENT_H_
#define TRIGGER_PLAN_CLIENT_H_

#include <string>
#include <memory>

#include "ros2_bdi_interfaces/msg/bdi_plan.hpp"
#include "ros2_bdi_interfaces/srv/bdi_plan_execution.hpp"

#include "rclcpp/rclcpp.hpp"

class TriggerPlanClient
{
    public:
        /* Constructor for the supporting node for calling the plan_execution service */
        TriggerPlanClient(const std::string& nodeBasename);
        
        /* Return true if operation is successful */
        bool triggerPlanExecution(const ros2_bdi_interfaces::msg::BDIPlan& bdiPlan);

        /* Return true if operation is successful */
        bool abortPlanExecution(const ros2_bdi_interfaces::msg::BDIPlan& bdiPlan);

    private:

        /* 
            Manage the request call toward the plan_execution service, so that the public functions
            for triggering/aborting plan execution are just wrappers for it avoiding code duplication
        */
        bool makePlanExecutionRequest(const ros2_bdi_interfaces::srv::BDIPlanExecution::Request::SharedPtr& request);

        // node to be spinned while making request to the plan_execution srv 
        rclcpp::Node::SharedPtr caller_node_;

        // client instance to make the request to the plan_execution srv
        rclcpp::Client<ros2_bdi_interfaces::srv::BDIPlanExecution>::SharedPtr caller_client_;
};

#endif //TRIGGER_PLAN_CLIENT_H_