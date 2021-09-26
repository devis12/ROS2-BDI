#ifndef COMMUNICATIONS_CLIENT_H_
#define COMMUNICATIONS_CLIENT_H_

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/srv/check_belief.hpp"
#include "ros2_bdi_interfaces/srv/upd_belief_set.hpp"
#include "ros2_bdi_interfaces/srv/check_desire.hpp"
#include "ros2_bdi_interfaces/srv/upd_desire_set.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"

#include "rclcpp/rclcpp.hpp"

using ros2_bdi_interfaces::msg::Belief;            
using ros2_bdi_interfaces::msg::BeliefSet;            
using ros2_bdi_interfaces::msg::Desire;  
using ros2_bdi_interfaces::srv::CheckBelief;  
using ros2_bdi_interfaces::srv::UpdBeliefSet;  
using ros2_bdi_interfaces::srv::CheckDesire;  
using ros2_bdi_interfaces::srv::UpdDesireSet; 

typedef struct{
  Belief belief;
  bool accepted;
  bool found;
} CheckBeliefResult;

typedef struct{
  Desire desire;
  bool accepted;
  bool found;
} CheckDesireResult;

typedef enum {ADD, DEL} UpdOperation;

typedef struct{
  Belief belief;
  UpdOperation op;
  bool accepted;
  bool performed;
} UpdBeliefResult;

typedef struct{
  Desire desire;
  UpdOperation op;
  bool accepted;
  bool performed;
} UpdDesireResult;

class CommunicationsClient
{
    public:
        CommunicationsClient()
        {
            node_ = rclcpp::Node::make_shared("communications_client");
        }

        CheckBeliefResult checkBeliefRequest(const string& agentRef, const string& agentGroup, const Belief& belief)
        {
            string serviceName = "/" + agentRef + "/check_belief_srv";
            CheckBeliefResult res{belief, false, false};

            try{
                ck_belief_client_ = node_->create_client<CheckBelief>(serviceName);
            
                while (!ck_belief_client_->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        return res;
                    }

                    RCLCPP_ERROR_STREAM(
                        node_->get_logger(),
                        ck_belief_client_->get_service_name() <<
                            " service client: waiting for service to appear...");
                }
                auto request = std::make_shared<CheckBelief::Request>();
                request->belief = belief;
                request->agent_group = agentGroup;
                auto future_result = ck_belief_client_->async_send_request(request);

                if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
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

        UpdBeliefResult updBeliefRequest(const string& agentRef, const string& agentGroup, const Belief& belief, const UpdOperation& op)
        {
            string serviceName = "/" + agentRef + "/" + 
                ((op==ADD)? "add" : "del") + 
                "_belief_srv";
            UpdBeliefResult res{belief, op, false, false};

            try{
                upd_belief_client_ = node_->create_client<UpdBeliefSet>(serviceName);

                while (!upd_belief_client_->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        return res;
                    }

                    RCLCPP_ERROR_STREAM(
                        node_->get_logger(),
                        upd_belief_client_->get_service_name() <<
                            " service client: waiting for service to appear...");
                }

                auto request = std::make_shared<UpdBeliefSet::Request>();
                request->belief = belief;
                request->agent_group = agentGroup;
                auto future_result = upd_belief_client_->async_send_request(request);

                if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
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
        
        CheckDesireResult checkDesireRequest(const string& agentRef, const string& agentGroup, const Desire& desire)
        {
            string serviceName = "/" + agentRef + "/check_desire_srv";
            CheckDesireResult res{desire, false, false};

            try{
                ck_desire_client_ = node_->create_client<CheckDesire>(serviceName);

                
                while (!ck_desire_client_->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        return res;
                    }

                    RCLCPP_ERROR_STREAM(
                        node_->get_logger(),
                        ck_desire_client_->get_service_name() <<
                            " service client: waiting for service to appear...");
                }

                auto request = std::make_shared<CheckDesire::Request>();
                request->desire = desire;
                request->agent_group = agentGroup;
                auto future_result = ck_desire_client_->async_send_request(request);

                if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
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

        UpdDesireResult updDesireRequest(const string& agentRef, const string& agentGroup, const Desire& desire, const UpdOperation& op)
        {
            string serviceName = "/" + agentRef + "/" + 
                ((op==ADD)? "add" : "del") + 
                "_desire_srv";
            UpdDesireResult res{desire, op, false, false};
            
            try{
                upd_desire_client_ = node_->create_client<UpdDesireSet>(serviceName);

                while (!upd_desire_client_->wait_for_service(std::chrono::seconds(1))) {
                    if (!rclcpp::ok()) {
                        return res;
                    }

                    RCLCPP_ERROR_STREAM(
                        node_->get_logger(),
                        upd_desire_client_->get_service_name() <<
                            " service client: waiting for service to appear...");
                }

                auto request = std::make_shared<UpdDesireSet::Request>();
                request->desire = desire;
                request->agent_group = agentGroup;
                auto future_result = upd_desire_client_->async_send_request(request);

                if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
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

    private:
        rclcpp::Node::SharedPtr node_;

        rclcpp::Client<CheckBelief>::SharedPtr ck_belief_client_;
        rclcpp::Client<CheckDesire>::SharedPtr ck_desire_client_;
        rclcpp::Client<UpdBeliefSet>::SharedPtr upd_belief_client_;
        rclcpp::Client<UpdDesireSet>::SharedPtr upd_desire_client_;
};

#endif  // COMMUNICATIONS_CLIENT_H_