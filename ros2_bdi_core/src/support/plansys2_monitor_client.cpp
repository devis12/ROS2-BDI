// header file for PlanSys2 Monitor Support clients node
#include "ros2_bdi_core/support/plansys2_monitor_client.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for PlanSys2 Monitor node
#include "ros2_bdi_core/params/plansys2_monitor_params.hpp"

using std::string;
using std::vector;

using lifecycle_msgs::srv::GetState;

PlanSys2MonitorClient::PlanSys2MonitorClient()
{   
    /*Init caller nodes*/
    caller_nodes_ = vector<rclcpp::Node::SharedPtr>();
    for(int i=0; i<PSYS2NODES; i++)
        caller_nodes_.push_back(rclcpp::Node::make_shared("plansys2_state_caller_"+i));
    
    /*Init caller clients*/
    caller_clients_ = vector<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>();
    string domain_expert_name = PSYS2_DOM_EXPERT;
    caller_clients_.push_back(caller_nodes_[PSYS2_DOM_EXPERT_I]->create_client<lifecycle_msgs::srv::GetState>(domain_expert_name + "/" + PSYS2_CK_STATE_SRV));  
    string problem_expert_name = PSYS2_PROB_EXPERT;
    caller_clients_.push_back(caller_nodes_[PSYS2_PROB_EXPERT_I]->create_client<lifecycle_msgs::srv::GetState>(problem_expert_name + "/" + PSYS2_CK_STATE_SRV));  
    string planner_name = PSYS2_PLANNER;
    caller_clients_.push_back(caller_nodes_[PSYS2_PLANNER_I]->create_client<lifecycle_msgs::srv::GetState>(planner_name + "/" + PSYS2_CK_STATE_SRV));  
    string executor_name = PSYS2_EXECUTOR;
    caller_clients_.push_back(caller_nodes_[PSYS2_EXECUTOR_I]->create_client<lifecycle_msgs::srv::GetState>(executor_name + "/" + PSYS2_CK_STATE_SRV));  
}

/* Get the reference to the node caller instance for the PlanSys2 node @psys2NodeName */
rclcpp::Node::SharedPtr PlanSys2MonitorClient::getCallerNode(const std::string& psys2NodeName)
{
    if(psys2NodeName == PSYS2_DOM_EXPERT && caller_nodes_.size() > PSYS2_DOM_EXPERT_I)
    {
        return caller_nodes_[PSYS2_DOM_EXPERT_I];
    }

    if(psys2NodeName == PSYS2_PROB_EXPERT && caller_nodes_.size() > PSYS2_PROB_EXPERT_I)
    {
        return caller_nodes_[PSYS2_PROB_EXPERT_I];
    }

    if(psys2NodeName == PSYS2_PLANNER && caller_nodes_.size() > PSYS2_PLANNER_I)
    {
        return caller_nodes_[PSYS2_PLANNER_I];
    }

    if(psys2NodeName == PSYS2_EXECUTOR && caller_nodes_.size() > PSYS2_EXECUTOR_I)
    {
        return caller_nodes_[PSYS2_EXECUTOR_I];
    }

    return {}; //unmatched
}

/* Get the reference to the client caller instance for the PlanSys2 node @psys2NodeName */
rclcpp::Client<GetState>::SharedPtr PlanSys2MonitorClient::getCallerClient(const std::string& psys2NodeName)
{
    if(psys2NodeName == PSYS2_DOM_EXPERT && caller_clients_.size() > PSYS2_DOM_EXPERT_I)
    {
        return caller_clients_[PSYS2_DOM_EXPERT_I];
    }

    if(psys2NodeName == PSYS2_PROB_EXPERT && caller_clients_.size() > PSYS2_PROB_EXPERT_I)
    {
        return caller_clients_[PSYS2_PROB_EXPERT_I];
    }

    if(psys2NodeName == PSYS2_PLANNER && caller_clients_.size() > PSYS2_PLANNER_I)
    {
        return caller_clients_[PSYS2_PLANNER_I];
    }

    if(psys2NodeName == PSYS2_EXECUTOR && caller_clients_.size() > PSYS2_EXECUTOR_I)
    {
        return caller_clients_[PSYS2_EXECUTOR_I];
    }

    return {}; //unmatched
}

/* Return true if {psys2NodeName}/get_state service called confirm that the node is active */
bool PlanSys2MonitorClient::isPsys2NodeActive(const std::string& psys2NodeName)
{
    rclcpp::Node::SharedPtr node = getCallerNode(psys2NodeName);
    rclcpp::Client<GetState>::SharedPtr client = getCallerClient(psys2NodeName);

    try{
    
        while (!client->wait_for_service(std::chrono::seconds(WAIT_SRV_UP))) {
            if (!rclcpp::ok()) {
                return false;
            }
            RCLCPP_ERROR_STREAM(
                node->get_logger(),
                client->get_service_name() <<
                    " service client: waiting for service to appear...");
        }

        auto request = std::make_shared<GetState::Request>();
        auto future_result = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, future_result, std::chrono::seconds(WAIT_RESPONSE_TIMEOUT)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return false;
        }

        auto response = future_result.get();
        return response->current_state.id == 3 && response->current_state.label == "active";
    
    }
    catch(const rclcpp::exceptions::RCLError& rclerr)
    {
        RCLCPP_ERROR(node->get_logger(), rclerr.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Response error in while trying to call get_state srv for %s", psys2NodeName);
    }

    return false;
}