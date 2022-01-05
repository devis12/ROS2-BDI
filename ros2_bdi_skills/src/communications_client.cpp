#include "ros2_bdi_skills/communications_client.hpp"

// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 communications nodes (get services' names)
#include "ros2_bdi_core/params/communications_params.hpp"

//seconds to wait before giving up on performing any request (service does not appear to be up)
#define WAIT_SRV_UP 1   

//seconds to wait before giving up on waiting for the response
#define WAIT_RESPONSE_TIMEOUT 1

using std::string;

using ros2_bdi_interfaces::msg::Belief;                
using ros2_bdi_interfaces::msg::Desire;  
using ros2_bdi_interfaces::srv::CheckBelief;  
using ros2_bdi_interfaces::srv::UpdBeliefSet;  
using ros2_bdi_interfaces::srv::CheckDesire;  
using ros2_bdi_interfaces::srv::UpdDesireSet; 

using BDICommunications::CommunicationsClient;
using BDICommunications::UpdOperation;
using BDICommunications::CheckBeliefResult;
using BDICommunications::CheckDesireResult;
using BDICommunications::UpdBeliefResult;
using BDICommunications::UpdDesireResult;

CommunicationsClient::CommunicationsClient()
{
    // node to perform async request to communication services of queried agent(s)
    // make the node spin just while waiting response or until timeout is reached
    node_ = rclcpp::Node::make_shared("communications_client");
}

CheckBeliefResult CommunicationsClient::checkBeliefRequest(const string& agent_ref, const string& agent_group, const Belief& belief)
{
    string serviceName = "/" + agent_ref + "/" + CK_BELIEF_SRV;
    CheckBeliefResult res{belief, false, false};

    try{
        ck_belief_client_ = node_->create_client<CheckBelief>(serviceName);
    
        while (!ck_belief_client_->wait_for_service(std::chrono::seconds(WAIT_SRV_UP))) {
            RCLCPP_ERROR_STREAM(
                node_->get_logger(),
                ck_belief_client_->get_service_name() <<
                    " service appears to be down");
                    
            return res;
        }
        auto request = std::make_shared<CheckBelief::Request>();
        request->belief = belief;
        request->agent_group = agent_group;
        auto future_result = ck_belief_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(WAIT_RESPONSE_TIMEOUT)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return res;
        }

        auto response = future_result.get();
        res.accepted = response->accepted;
        res.found = response->found;
    
    }
    catch(const rclcpp::exceptions::RCLError& rclerr)
    {
        RCLCPP_ERROR(node_->get_logger(), rclerr.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Response error in " + serviceName);
    }

    return res;
}

UpdBeliefResult CommunicationsClient::updBeliefRequest(const string& agent_ref, const string& agent_group, const Belief& belief, const UpdOperation& op)
{
    string serviceName = "/" + agent_ref + "/";
    if(op == ADD)
        serviceName += ADD_BELIEF_SRV;
    else
        serviceName += DEL_BELIEF_SRV;

    UpdBeliefResult res{belief, op, false, false};

    try{
        upd_belief_client_ = node_->create_client<UpdBeliefSet>(serviceName);

        while (!upd_belief_client_->wait_for_service(std::chrono::seconds(WAIT_SRV_UP))) {
            RCLCPP_ERROR_STREAM(
                node_->get_logger(),
                upd_belief_client_->get_service_name() <<
                    " service appears to be down");
                    
            return res;
        }

        auto request = std::make_shared<UpdBeliefSet::Request>();
        request->belief = belief;
        request->agent_group = agent_group;
        auto future_result = upd_belief_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(WAIT_RESPONSE_TIMEOUT)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return res;
        }

        auto response = future_result.get();
        res.accepted = response->accepted;
        res.performed = response->updated;
    }
    catch(const rclcpp::exceptions::RCLError& rclerr)
    {
        RCLCPP_ERROR(node_->get_logger(), rclerr.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Response error in " + serviceName);
    }

    return res;
}

CheckDesireResult CommunicationsClient::checkDesireRequest(const string& agent_ref, const string& agent_group, const Desire& desire)
{
    string serviceName = "/" + agent_ref + "/" + CK_DESIRE_SRV;
    CheckDesireResult res{desire, false, false};

    try{
        ck_desire_client_ = node_->create_client<CheckDesire>(serviceName);
        
        while (!ck_desire_client_->wait_for_service(std::chrono::seconds(WAIT_SRV_UP))) {
            RCLCPP_ERROR_STREAM(
                node_->get_logger(),
                ck_desire_client_->get_service_name() <<
                    " service appears to be down");
                    
            return res;
        }

        auto request = std::make_shared<CheckDesire::Request>();
        request->desire = desire;
        request->agent_group = agent_group;
        auto future_result = ck_desire_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(WAIT_RESPONSE_TIMEOUT)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return res;
        }

        auto response = future_result.get();
        res.accepted = response->accepted;
        res.found = response->found;
    }
    catch(const rclcpp::exceptions::RCLError& rclerr)
    {
        RCLCPP_ERROR(node_->get_logger(), rclerr.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Response error in " + serviceName);
    }
    
    return res;
}

UpdDesireResult CommunicationsClient::updDesireRequest(const string& agent_ref, const string& agent_group, const Desire& desire, const UpdOperation& op)
{
    string serviceName = "/" + agent_ref + "/";
    if(op == ADD)
        serviceName += ADD_DESIRE_SRV;
    else
        serviceName += DEL_DESIRE_SRV;

    UpdDesireResult res{desire, op, false, false};
    
    try{
        upd_desire_client_ = node_->create_client<UpdDesireSet>(serviceName);

        while (!upd_desire_client_->wait_for_service(std::chrono::seconds(WAIT_SRV_UP))) {
            RCLCPP_ERROR_STREAM(
                node_->get_logger(),
                upd_desire_client_->get_service_name() <<
                    " service appears to be down");
                    
            return res;
        }

        auto request = std::make_shared<UpdDesireSet::Request>();
        request->desire = desire;
        request->agent_group = agent_group;
        auto future_result = upd_desire_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(WAIT_RESPONSE_TIMEOUT)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return res;
        }

        auto response = future_result.get();
        res.accepted = response->accepted;
        res.performed = response->updated;
    }
    catch(const rclcpp::exceptions::RCLError& rclerr)
    {
        RCLCPP_ERROR(node_->get_logger(), rclerr.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Response error in " + serviceName);
    }

    return res;
}
