#include "ros2_bdi_on_litter_world/recycle.hpp"
#include "ros2_bdi_on_litter_world/params.hpp"

using std::string;
using std::placeholders::_1;
using std::placeholders::_2;

using litter_world_interfaces::action::CmdLoad;
using GoalHandleCmdLoad = rclcpp_action::ClientGoalHandle<CmdLoad>;


Recycle::Recycle(const string& litter_type)
: BDIActionExecutor("recycle_"+litter_type, 2), litter_type_(litter_type)
{
    string robot_name = this->get_parameter("agent_id").as_string();
    litter_unload_action_name_ = "/cmd_" + robot_name + "_hold";
    this->client_cmd_unload_ptr_ = rclcpp_action::create_client<CmdLoad>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        litter_unload_action_name_);
    
}


void Recycle::sendGoal()
{
    if (!this->client_cmd_unload_ptr_->wait_for_action_server(std::chrono::seconds(LWACTIONS_WAIT_TIMEOUT))) {
        RCLCPP_ERROR(this->get_logger(), "Action server " + litter_unload_action_name_ + " not available after waiting");
        rclcpp::shutdown();
        return;
    }

    auto goal_msg = CmdLoad::Goal();
    goal_msg.cmd = "unload";

    auto send_goal_options = rclcpp_action::Client<CmdLoad>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&Recycle::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&Recycle::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Recycle::result_callback, this, _1);
    this->client_cmd_unload_ptr_->async_send_goal(goal_msg, send_goal_options);
    goal_sent_ = true;
}

void Recycle::goal_response_callback(std::shared_future<GoalHandleCmdLoad::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) 
        execFailed("Goal was rejected!");
    else
        goal_accepted_ = true;
}

void Recycle::feedback_callback(
    GoalHandleCmdLoad::SharedPtr,
    const std::shared_ptr<const CmdLoad::Feedback> feedback_msg)
{
    //TODO progress
}

void Recycle::result_callback(const GoalHandleCmdLoad::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            execFailed(); //RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            execFailed(); //RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            execFailed(); //RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
    
    if(result.result->performed)
        execSuccess("Recycled one item of " + litter_type_);
    else
        execFailed();
}

