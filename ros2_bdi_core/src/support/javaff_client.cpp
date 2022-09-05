/*  Header for supporting plan client trigger node to make call launching/aborting a plan execution*/
#include "ros2_bdi_core/support/javaff_client.hpp"
/* Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Scheduler node (timeout for srv)*/
#include "ros2_bdi_core/params/scheduler_params.hpp"

using std::string;

using BDIManaged::ManagedDesire;

using javaff_interfaces::srv::JavaFFPlan;
using javaff_interfaces::srv::UnexpectedState;


JavaFFClient::JavaFFClient(const string& nodeBasename)
{
    caller_node_ = rclcpp::Node::make_shared(nodeBasename);
    start_plan_client_ = caller_node_->create_client<JavaFFPlan>(JAVAFF_START_PLAN_SRV);
    unexpected_state_client_ = caller_node_->create_client<UnexpectedState>(JAVAFF_UNEXPECTED_STATE_SRV);
}

bool JavaFFClient::launchPlanSearch(const ros2_bdi_interfaces::msg::Desire& fulfilling_desire, const string& problem, const int& interval)
{
    auto req = std::make_shared<JavaFFPlan::Request>();
    req->fulfilling_desire = fulfilling_desire;
    req->problem = problem;
    req->search_interval = interval;
    return makePlanSearchRequest(req);
}

/* 
    Manage the request call toward the javaff start_plan service
*/
bool JavaFFClient::makePlanSearchRequest(const JavaFFPlan::Request::SharedPtr& request)
{
    try{
        while (!start_plan_client_->wait_for_service(std::chrono::seconds(WAIT_SRV_UP))) {
            if (!rclcpp::ok()) {
                return false;
            }
            RCLCPP_ERROR_STREAM(
                caller_node_->get_logger(),
                start_plan_client_->get_service_name() <<
                    " service client: waiting for service to appear...");
        }
        auto future_result = start_plan_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(caller_node_, future_result, std::chrono::seconds(WAIT_RESPONSE_TIMEOUT)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return false;
        }

        auto response = future_result.get();
        return response->accepted;
    
    }
    catch(const rclcpp::exceptions::RCLError& rclerr)
    {
        RCLCPP_ERROR(caller_node_->get_logger(), rclerr.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(caller_node_->get_logger(), "Response error in while trying to call %s srv", JAVAFF_START_PLAN_SRV);
    }
    
    return false;
}

bool JavaFFClient::callUnexpectedStateSrv(const string& pddl_problem)
{
    auto req = std::make_shared<UnexpectedState::Request>();
    req->pddl_problem = pddl_problem;
    return makeUnexpectedStateRequest(req);
}

/* 
    Manage the request call toward the javaff unexpected_state service
*/
bool JavaFFClient::makeUnexpectedStateRequest(const UnexpectedState::Request::SharedPtr& request)
{
    try{
        while (!unexpected_state_client_->wait_for_service(std::chrono::seconds(WAIT_SRV_UP))) {
            if (!rclcpp::ok()) {
                return false;
            }
            RCLCPP_ERROR_STREAM(
                caller_node_->get_logger(),
                unexpected_state_client_->get_service_name() <<
                    " service client: waiting for service to appear...");
        }
        auto future_result = unexpected_state_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(caller_node_, future_result, std::chrono::seconds(WAIT_RESPONSE_TIMEOUT)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return false;
        }

        auto response = future_result.get();
        return response->handled;
    
    }
    catch(const rclcpp::exceptions::RCLError& rclerr)
    {
        RCLCPP_ERROR(caller_node_->get_logger(), rclerr.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(caller_node_->get_logger(), "Response error in while trying to call %s srv", JAVAFF_UNEXPECTED_STATE_SRV);
    }
    
    return false;
}