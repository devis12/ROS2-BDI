#ifndef BDI_ACTION_EXECUTOR_H_
#define BDI_ACTION_EXECUTOR_H_

#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>
#include <set>
#include <mutex>
#include <thread>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/srv/upd_belief_set.hpp"
#include "ros2_bdi_interfaces/srv/upd_desire_set.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"

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
using std::mutex;
using std::optional;

using ros2_bdi_interfaces::msg::Belief;            
using ros2_bdi_interfaces::msg::BeliefSet;            
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

        // flag to know if last belief response is arrived
        last_belief_req_resp_arrived_ = false;
        last_desire_req_resp_arrived_ = false;

        // flag to get info of last belief request
        last_belief_req_accepted_ = false;
        // flag to get info about last desire request
        last_desire_req_accepted_ = false;

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

    /*return arguments of the actions*/
    vector<string> getArguments() {return get_arguments();}
    /*return current progress state of the action*/
    float getProgress() {return progress_;} 
    
    bool lastBeliefReqResponseArrived(){return last_belief_req_resp_arrived_;}
    bool lastDesireReqResponseArrived(){return last_desire_req_resp_arrived_;}

    bool lastBeliefReqAccepted(){return last_belief_req_accepted_;}
    bool lastDesireReqAccepted(){return last_desire_req_accepted_;}
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
    void sendBeliefRequest(const string& agentRef, const Belief& belief, const Operation& op)
    {
      last_belief_req_resp_arrived_ = false;

      shared_ptr<thread> sendBeliefReqThread = std::make_shared<thread>(
          bind(&BDIActionExecutor::sendBeliefRequestClientThread, this, agentRef, belief, op));
      sendBeliefReqThread->detach();
    }


    void sendBeliefRequestClientThread(const string& agentRef, const Belief& belief, const Operation& op)
    {
       string serviceName = "/" + agentRef + "/" + 
        ((op == ADD)? "add_belief_srv" : "del_belief_srv");
       
       bool accepted = false;

       try{
            //check for service to be up
            rclcpp::Client<UpdBeliefSet>::SharedPtr client_ = this->create_client<UpdBeliefSet>(serviceName);
            if(!client_->wait_for_service(std::chrono::seconds(1)))
            {
                //service seems not even present, just return false
                RCLCPP_WARN(this->get_logger(), serviceName + " server does not appear to be up");
                accepted = false;
            }
            else
            {
              auto request = std::make_shared<UpdBeliefSet::Request>();
              request->belief = belief;
              request->agent_group = agent_group_;
              auto future = client_->async_send_request(request);
              auto response = future.get();

              accepted = response->accepted;
            }
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

        last_belief_req_accepted_ = accepted;
        last_belief_req_resp_arrived_ = true;
    }



    /*
      Utility methods to send desire request (ADD/DEL) to other agents within the action doWork method implementation
    */
    void sendDesireRequest(const string& agentRef, const Desire& desire, const Operation& op, const bool& monitorFulfill)
    {
      last_desire_req_resp_arrived_ = false;
      shared_ptr<thread> sendDesireReqThread = std::make_shared<thread>(
          bind(&BDIActionExecutor::sendDesireRequestClientThread, this, agentRef, desire, op, monitorFulfill));
      sendDesireReqThread->detach();//wait for thread to terminate and get the response

    }

    void sendDesireRequestClientThread(const string& agentRef, const Desire& desire, const Operation& op, const bool& monitorFulfill)
    {
       string serviceName = "/" + agentRef + "/" + 
        ((op == ADD)? "add_desire_srv" : "del_desire_srv");

       bool accepted = false;

       try{
            //check for service to be up
            rclcpp::Client<UpdDesireSet>::SharedPtr client_ = this->create_client<UpdDesireSet>(serviceName);
            if(!client_->wait_for_service(std::chrono::seconds(1)))
            {
                //service seems not even present, just return false
                RCLCPP_WARN(this->get_logger(), serviceName + " server does not appear to be up");
                accepted = false;
            }
            else
            {
              auto request = std::make_shared<UpdDesireSet::Request>();
              request->desire = desire;
              request->agent_group = agent_group_;

              auto future = client_->async_send_request(request);
              auto response = future.get();


              bool accepted = response->accepted;
              if(accepted && monitorFulfill)//desire request accepted and monitor fulfillment has been requested
                monitor(agentRef, desire);
            }
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

        last_desire_req_accepted_ = accepted;
        last_desire_req_resp_arrived_ = true;
    }

    /*
      if no monitored desire, just return false
      otherwise check if it is fulfilled in the respective monitored belief set
    */
    bool isMonitoredDesireSatisfied()
    {
      if(monitored_desire_ == ManagedDesire{})
        return false;
      else
        return monitored_desire_.isFulfilled(monitored_beliefset_);
    }

    // method to be called when the execution successfully comes to completion (generic success msg added)
    void execSuccess()
    { 
      execSuccess("");
    }

    // method to be called when the execution successfully comes to completion (specific success msg added)
    void execSuccess(const string& statusSpecific)
    {
      RCLCPP_INFO(this->get_logger(), "Action execution success: " + statusSpecific);
      finish(true, 1.0, action_name_ + " successful execution" + ((statusSpecific == "")? ": action performed" : ": " + statusSpecific));
    }

    //method to be called when the execution fail (generic error to be given: no specific added)
    void execFailed()
    { 
      execFailed("");
    }

    //method to be called when the execution fail (specific error to be given)
    void execFailed(const string& errSpecific)
    {
      RCLCPP_ERROR(this->get_logger(), "Action execution failed: " + errSpecific);
      finish(false, progress_, action_name_ + " failed execution" + ((errSpecific == "")? ": generic error" : ": " + errSpecific));
    }


private:

    /*
      Monitor belief set update of an agent 
    */
    void monitor(const string& agentRef, const Desire& desire)
    {
      rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
      qos_keep_all.keep_all();

      agent_belief_set_subscriber_ = this->create_subscription<BeliefSet>(
                "/"+agentRef+"/belief_set", qos_keep_all,
                bind(&BDIActionExecutor::agentBeliefSetCallback, this, _1));
      
      monitored_desire_ = ManagedDesire{desire};
    }

    /*
      update the current monitored belief set 
    */
    void agentBeliefSetCallback(const BeliefSet::SharedPtr msg)
    {
       monitored_beliefset_ = BDIFilter::extractMGBeliefs(msg->value);
    }


    //listen to belief set of agents to know if/when desires are fulfilled
    rclcpp::Subscription<BeliefSet>::SharedPtr agent_belief_set_subscriber_;

    //currently monitoring desire
    ManagedDesire monitored_desire_;

    //currently monitoring belief set
    set<ManagedBelief> monitored_beliefset_;

    // action name
    string action_name_;

    // action params
    vector<string> action_params_;

    // agent id that defines the namespace in which the node operates
    string agent_id_;

    // agent group id that defines the group of agents it is a part of (used to decide which belief/desire to accept or discard)
    string agent_group_;

    // last belief request response arrived
    bool last_belief_req_resp_arrived_;
    // last desire request response arrived
    bool last_desire_req_resp_arrived_;

    // last belief request accepted flag
    bool last_belief_req_accepted_;

    // last desire request accepted flag
    bool last_desire_req_accepted_;

    // progress of the current action execution
    float progress_;
};

#endif  // BDI_ACTION_EXECUTOR_H_
