#ifndef BDI_ACTION_EXECUTOR_H_
#define BDI_ACTION_EXECUTOR_H_

#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>
#include <thread>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/srv/upd_belief_set.hpp"
#include "ros2_bdi_interfaces/srv/upd_desire_set.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define PARAM_AGENT_ID "agent_id"
#define PARAM_AGENT_GROUP_ID "agent_group"
#define PARAM_DEBUG "debug"

using std::string;
using std::vector;
using std::shared_ptr;
using std::thread;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::optional;

using ros2_bdi_interfaces::msg::Belief;            
using ros2_bdi_interfaces::msg::Desire;  
using ros2_bdi_interfaces::srv::UpdBeliefSet;  
using ros2_bdi_interfaces::srv::UpdDesireSet;  

//Operation that can be performed when sending Belief/Desire requests
typedef enum {ADD, DEL} Operation;

class BDIActionExecutor : public plansys2::ActionExecutorClient
{
public:
  BDIActionExecutor(const string& action_name, const int& working_freq) : 
    plansys2::ActionExecutorClient(action_name, milliseconds((int) (1000/working_freq)))
    {
        this->declare_parameter(PARAM_AGENT_ID, "agent0");
        this->declare_parameter(PARAM_AGENT_GROUP_ID, "agent0_group");
        this->declare_parameter(PARAM_DEBUG, true);

        // store action name
        action_name_ = action_name;
        
        // agent's namespace
        agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

        // agent's group name
        agent_group_ = this->get_parameter(PARAM_AGENT_GROUP_ID).as_string();

        // set agent id as specialized arguments
        vector<string> specialized_arguments = vector<string>();
        specialized_arguments.push_back(agent_id_);

        this->set_parameter(rclcpp::Parameter("action_name", action_name_));
        this->set_parameter(rclcpp::Parameter("specialized_arguments", specialized_arguments));

        // action node, once created, must pass to inactive state to be ready to execute. 
      // ActionExecutorClient is a managed node (https://design.ros2.org/articles/node_lifecycle.html)
      this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  }

       rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    init();

    return ActionExecutorClient::on_activate(previous_state);
  }

protected:
    /*
        Init to call at the start, after construction method, to get the node actually started
        Main thing to be added: frequency at which to perform doWork method which publish the
        progress feedback
    */
    void init()
    { 
        // status set to initial status 0.0%
        progress_ = 0.0f;

        RCLCPP_INFO(this->get_logger(), "Action executor controller for \"" + action_name_ + "\" ready for execution");

    }

    void do_work()
    {
      action_params_ = get_arguments();//get arguments of action execution

      progress_ += advanceWork();
      if(progress_ >= 1.0)
        execSuccess();//send feedback of complete execution to plansys2 executor (node deactivate again)

      else
      {
        float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
        string progress_100S = std::to_string(progress_100);
        progress_100S = progress_100S.substr(0, progress_100S.find(".", 0)+2);//just save one decimal

        string params_string = "";
        for(int i = 0; i < action_params_.size(); i++)
          params_string += " " + action_params_[i];
        
        //feedback string to be sent to plansys2 executor along with the progress_status (logged if debug=TRUE)
        string feedback_string = string("[" + action_name_ +  params_string
             + "] "+ progress_100S + "%%");
        send_feedback(progress_, feedback_string);//send feedback to plansys2 executor

        if(this->get_parameter(PARAM_DEBUG).as_bool())
          RCLCPP_INFO(this->get_logger(), feedback_string);
      }
    }

    virtual float advanceWork() = 0;

    /*
      Utility methods to send belief request (ADD/DEL) to other agents within the action doWork method implementation
    */
    bool sendBeliefRequest(const string& agentRef, const Belief& belief, const Operation& op)
    {
      // boolean to verify if the request has been accepted
      bool acceptBeliefRequest = false;
      shared_ptr<thread> sendBeliefReqThread = std::make_shared<thread>(
          bind(&BDIActionExecutor::sendBeliefRequestClientThread, this, agentRef, belief, op, acceptBeliefRequest));
      sendBeliefReqThread->join();//wait for thread to terminate and get the response

      return acceptBeliefRequest;

    }


    void sendBeliefRequestClientThread(const string& agentRef, const Belief& belief, const Operation& op, bool& accepted)
    {
       string serviceName = agentRef + "/" + 
        ((op == ADD)? "add_belief_srv" : "del_belief_srv");

       try{
            //check for service to be up
            rclcpp::Client<UpdBeliefSet>::SharedPtr client_ = this->create_client<UpdBeliefSet>(serviceName);
            if(!client_->wait_for_service(std::chrono::seconds(1))){
                //service seems not even present, just return false
                RCLCPP_WARN(this->get_logger(), serviceName + " server does not appear to be up");
                accepted = false;
                return;
            }
            auto request = std::make_shared<UpdBeliefSet::Request>();
            request->belief = belief;
            request->agent_group = agent_group_;
            auto future = client_->async_send_request(request);
            auto response = future.get();

            accepted = response->accepted;
        }
        catch(const rclcpp::exceptions::RCLError& rclerr)
        {
            //TODO fix this by avoiding to create a new client if a
            RCLCPP_ERROR(this->get_logger(), rclerr.what());
            accepted = false;
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Response error in " + serviceName);
            accepted = false;
        }
    }



    /*
      Utility methods to send desire request (ADD/DEL) to other agents within the action doWork method implementation
    */
    bool sendDesireRequest(const string& agentRef, const Desire& desire, const Operation& op)
    {
      // boolean to verify if the request has been accepted
      bool acceptDesireRequest = false;
      shared_ptr<thread> sendDesireReqThread = std::make_shared<thread>(
          bind(&BDIActionExecutor::sendDesireRequestClientThread, this, agentRef, desire, op, acceptDesireRequest));
      sendDesireReqThread->join();//wait for thread to terminate and get the response

      return acceptDesireRequest;

    }

    void sendDesireRequestClientThread(const string& agentRef, const Desire& desire, const Operation& op, bool& accepted)
    {
       string serviceName = agentRef + "/" + 
        ((op == ADD)? "add_desire_srv" : "del_desire_srv");

       try{
            //check for service to be up
            rclcpp::Client<UpdDesireSet>::SharedPtr client_ = this->create_client<UpdDesireSet>(serviceName);
            if(!client_->wait_for_service(std::chrono::seconds(1))){
                //service seems not even present, just return false
                RCLCPP_WARN(this->get_logger(), serviceName + " server does not appear to be up");
                accepted = false;
                return;
            }
            auto request = std::make_shared<UpdDesireSet::Request>();
            request->desire = desire;
            request->agent_group = agent_group_;
            auto future = client_->async_send_request(request);
            auto response = future.get();

            accepted = response->accepted;
        }
        catch(const rclcpp::exceptions::RCLError& rclerr)
        {
            //TODO fix this by avoiding to create a new client if a
            RCLCPP_ERROR(this->get_logger(), rclerr.what());
            accepted = false;
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Response error in " + serviceName);
            accepted = false;
        }
    }

    // method to be called when the execution successfully comes to completion (generic success msg added)
    void execSuccess()
    { 
      execSuccess("");
    }

    // method to be called when the execution successfully comes to completion (specific success msg added)
    void execSuccess(const string& statusSpecific)
    {
      finish(true, progress_, action_name_ + " successful execution" + ((statusSpecific == "")? ": action performed" : ": " + statusSpecific));
    }

    //method to be called when the execution fail (generic error to be given: no specific added)
    void execFailed()
    { 
      execFailed("");
    }

    //method to be called when the execution fail (specific error to be given)
    void execFailed(const string& errSpecific)
    {
      finish(false, progress_, action_name_ + " failed execution" + ((errSpecific == "")? ": generic error" : ": " + errSpecific));
    }


private:

    // action name
    string action_name_;

    // action params
    vector<string> action_params_;

    // agent id that defines the namespace in which the node operates
    string agent_id_;

    // agent group id that defines the group of agents it is a part of (used to decide which belief/desire to accept or discard)
    string agent_group_;

    // progress of the current action execution
    float progress_;
};

#endif  // BDI_ACTION_EXECUTOR_H_
