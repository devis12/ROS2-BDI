/*  Header for supporting plan client trigger node to make call launching/aborting a plan execution*/
#include "ros2_bdi_core/support/trigger_plan_client.hpp"
/*  Header containing plan execution srv name*/
#include "ros2_bdi_core/params/plan_director_params.hpp"
/* Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Scheduler node (timeout for srv)*/
#include "ros2_bdi_core/params/scheduler_params.hpp"

using std::string;

using ros2_bdi_interfaces::msg::BDIPlan;
using ros2_bdi_interfaces::srv::BDIPlanExecution;

 /* Constructor for the supporting node for calling the plan_execution service */
TriggerPlanClient::TriggerPlanClient(const string& nodeBasename)
{
    caller_node_ = rclcpp::Node::make_shared(nodeBasename);
    caller_client_ = caller_node_->create_client<BDIPlanExecution>(PLAN_EXECUTION_SRV);
}

/* Return true if operation is successful */
bool TriggerPlanClient::triggerPlanExecution(const BDIPlan& bdiPlan)
{
    auto req = std::make_shared<BDIPlanExecution::Request>();
    req->plan = bdiPlan;
    req->request = req->EXECUTE;
    return makePlanExecutionRequest(req);
}

/* Return true if operation is successful */
bool TriggerPlanClient::abortPlanExecution(const BDIPlan& bdiPlan)
{
    auto req = std::make_shared<BDIPlanExecution::Request>();
    req->plan = bdiPlan;
    req->request = req->ABORT;
    return makePlanExecutionRequest(req);
}

/* 
    Manage the request call toward the plan_execution service, so that the public functions
    for triggering/aborting plan execution are just wrappers for it avoiding code duplication
*/
bool TriggerPlanClient::makePlanExecutionRequest(const BDIPlanExecution::Request::SharedPtr& request)
{
        try{
    
        while (!caller_client_->wait_for_service(std::chrono::seconds(WAIT_SRV_UP))) {
            if (!rclcpp::ok()) {
                return false;
            }
            RCLCPP_ERROR_STREAM(
                caller_node_->get_logger(),
                caller_client_->get_service_name() <<
                    " service client: waiting for service to appear...");
        }

        auto future_result = caller_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(caller_node_, future_result, std::chrono::seconds(WAIT_RESPONSE_TIMEOUT)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return false;
        }

        auto response = future_result.get();
        return response->success;
    
    }
    catch(const rclcpp::exceptions::RCLError& rclerr)
    {
        RCLCPP_ERROR(caller_node_->get_logger(), rclerr.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(caller_node_->get_logger(), "Response error in while trying to call %s srv", PLAN_EXECUTION_SRV);
    }
    
    return false;
}