/*  Header for supporting plan client trigger node to make call launching/aborting a plan execution*/
#include "ros2_bdi_core/support/javaff_client.hpp"
/* Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Scheduler node (timeout for srv)*/
#include "ros2_bdi_core/params/scheduler_params.hpp"

using std::string;

using BDIManaged::ManagedDesire;

using javaff_interfaces::srv::JavaFFPlan;


JavaFFClient::JavaFFClient(const string& nodeBasename)
{
    caller_node_ = rclcpp::Node::make_shared(nodeBasename);
    start_plan_client_ = caller_node_->create_client<JavaFFPlan>(JAVAFF_START_PLAN_SRV);
}

bool JavaFFClient::launchPlanSearch(const string& problem, const int& interval)
{
    auto req = std::make_shared<JavaFFPlan::Request>();
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
        while (!start_plan_client_->wait_for_service(std::chrono::seconds(WAIT_PLAN_EXEC_SRV_UP))) {
            if (!rclcpp::ok()) {
                return false;
            }
            RCLCPP_ERROR_STREAM(
                caller_node_->get_logger(),
                start_plan_client_->get_service_name() <<
                    " service client: waiting for service to appear...");
        }
        auto future_result = start_plan_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(caller_node_, future_result, std::chrono::seconds(WAIT_PLAN_EXEC_RESPONSE_TIMEOUT)) !=
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