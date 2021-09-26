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
#include "ros2_bdi_interfaces/srv/check_belief.hpp"
#include "ros2_bdi_interfaces/srv/upd_belief_set.hpp"
#include "ros2_bdi_interfaces/srv/check_desire.hpp"
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
using ros2_bdi_interfaces::srv::CheckBelief;  
using ros2_bdi_interfaces::srv::UpdBeliefSet;  
using ros2_bdi_interfaces::srv::CheckDesire;  
using ros2_bdi_interfaces::srv::UpdDesireSet;  

typedef struct{
  Belief belief;
  bool arrived;
  bool accepted;
  bool found;
} CheckBeliefResult;

typedef struct{
  Desire desire;
  bool arrived;
  bool accepted;
  bool found;
} CheckDesireResult;

//Operation that can be performed when sending Belief/Desire requests
typedef enum {ADD, DEL} UpdOperation;

typedef struct{
  Belief belief;
  UpdOperation op;
  bool arrived;
  bool accepted;
  bool performed;
} UpdBeliefResult;

typedef struct{
  Desire desire;
  UpdOperation op;
  bool arrived;
  bool accepted;
  bool performed;
} UpdDesireResult;

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
    activation();
    std::cout << "\n\n!!! (" << this->get_parameter("action_name").as_string() << ") activated!\n\n" << std::endl;
    return ActionExecutorClient::on_activate(previous_state);
  }

protected:

    /*return arguments of the actions*/
    vector<string> getArguments() {return get_arguments();}
    /*return current progress state of the action*/
    float getProgress() {return progress_;} 

    CheckBeliefResult  checkBeliefRequestStatus()  {return last_belief_ck_;}
    CheckDesireResult  checkDesireRequestStatus()  {return last_desire_ck_;}
    UpdBeliefResult updBeliefRequestStatus()    {return last_belief_upd_;}
    UpdDesireResult updDesireRequestStatus()    {return last_desire_upd_;}

    /*
        Call to perform at each action activation
    */
    void activation()
    { 
        // status set to initial status 0.0%
        progress_ = 0.0f;

        // flag to get info of last belief check request
        last_belief_ck_.belief = Belief(); 
        last_belief_ck_.arrived = false; 
        last_belief_ck_.accepted = false; 
        last_belief_ck_.found = false; 
        // flag to get info of last belief request
        last_belief_upd_.belief = Belief(); 
        last_belief_upd_.arrived = false;
        last_belief_upd_.accepted = false;
        last_belief_upd_.performed = false;

        // flag to get info of last desire read request
        last_desire_ck_.desire = Desire(); 
        last_desire_ck_.arrived = false; 
        last_desire_ck_.accepted = false; 
        last_desire_ck_.found = false; 
        // flag to get info about last desire request
        last_desire_upd_.desire = Desire(); 
        last_desire_upd_.arrived = false;
        last_desire_upd_.accepted = false;
        last_desire_upd_.performed = false;
      

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

    void sendCheckBeliefRequest(const string& agentRef, const Belief& belief)
    {
      shared_ptr<thread> sendBeliefReqThread = std::make_shared<thread>(
          bind(&BDIActionExecutor::checkBeliefRequestClientThread, this, agentRef, belief));
      sendBeliefReqThread->detach();
    }

    void checkBeliefRequestClientThread(const string& agentRef, const Belief& belief)
    {
      string serviceName = "/" + agentRef + "/check_belief_srv";
      
      last_belief_ck_.belief = belief;
      last_belief_ck_.arrived = false;
      last_belief_ck_.accepted = false;
      last_belief_ck_.found = false;
      
      try{
            //check for service to be up
            rclcpp::Client<CheckBelief>::SharedPtr client_ = this->create_client<CheckBelief>(serviceName);
            if(!client_->wait_for_service(std::chrono::seconds(1)))
            {
                //service seems not even present, just return false
                RCLCPP_WARN(this->get_logger(), serviceName + " server does not appear to be up");
            }
            else
            {
              auto request = std::make_shared<CheckBelief::Request>();
              request->belief = belief;
              request->agent_group = agent_group_;

              auto future = client_->async_send_request(request);
              auto response = future.get();

              last_belief_ck_.found = response->found;
              last_belief_ck_.accepted = response->accepted;
              last_belief_ck_.arrived = true;
            }
        }
        catch(const rclcpp::exceptions::RCLError& rclerr)
        {
            //TODO fix this by avoiding to create a new client if a
            RCLCPP_ERROR(this->get_logger(), rclerr.what());
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Response error in " + serviceName);
        }

        //mtx_desire_req_lock_.unlock();
    }

    /*
      Utility methods to send belief request (ADD/DEL) to other agents within the action doWork method implementation
    */
    void sendBeliefRequest(const string& agentRef, const Belief& belief, const UpdOperation& op)
    {

      shared_ptr<thread> sendBeliefReqThread = std::make_shared<thread>(
          bind(&BDIActionExecutor::updBeliefRequestClientThread, this, agentRef, belief, op));
      sendBeliefReqThread->detach();
    }


    void updBeliefRequestClientThread(const string& agentRef, const Belief& belief, const UpdOperation& op)
    {
        string serviceName = "/" + agentRef + "/" + 
          ((op==ADD)? "add" : "del") + 
          "_belief_srv";

        last_belief_upd_.belief = belief;
        last_belief_upd_.op = op;

        last_belief_upd_.arrived = false;
        last_belief_upd_.accepted = false;
        last_belief_upd_.performed = false;

       try{
            //check for service to be up
            rclcpp::Client<UpdBeliefSet>::SharedPtr client_ = this->create_client<UpdBeliefSet>(serviceName);
            if(!client_->wait_for_service(std::chrono::seconds(1)))
            {
                //service seems not even present, just return false
                RCLCPP_WARN(this->get_logger(), serviceName + " server does not appear to be up");
            }
            else
            {
              auto request = std::make_shared<UpdBeliefSet::Request>();
              request->belief = belief;
              request->agent_group = agent_group_;
              auto future = client_->async_send_request(request);
              auto response = future.get();

              last_belief_upd_.performed = response->updated;
              last_belief_upd_.accepted = response->accepted;
              last_belief_upd_.arrived = true;
            }
        }
        catch(const rclcpp::exceptions::RCLError& rclerr)
        {
            //TODO fix this by avoiding to create a new client if a
            RCLCPP_ERROR(this->get_logger(), rclerr.what());
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Response error in " + serviceName);
        }
    }

    void sendCheckDesireRequest(const string& agentRef, const Desire& desire)
    {
      shared_ptr<thread> sendDesireReqThread = std::make_shared<thread>(
          bind(&BDIActionExecutor::checkDesireRequestClientThread, this, agentRef, desire));
      sendDesireReqThread->detach();
    }

    void checkDesireRequestClientThread(const string& agentRef, const Desire& desire)
    {
      string serviceName = "/" + agentRef + "/check_desire_srv";
      
      last_desire_ck_.desire = desire;
      last_desire_ck_.arrived = false;
      last_desire_ck_.accepted = false;
      last_desire_ck_.found = false;
      
      try{
            //check for service to be up
            rclcpp::Client<CheckDesire>::SharedPtr client_ = this->create_client<CheckDesire>(serviceName);
            if(!client_->wait_for_service(std::chrono::seconds(1)))
            {
                //service seems not even present, just return false
                RCLCPP_WARN(this->get_logger(), serviceName + " server does not appear to be up");
            }
            else
            {
              auto request = std::make_shared<CheckDesire::Request>();
              request->desire = desire;
              request->agent_group = agent_group_;

              auto future = client_->async_send_request(request);
              auto response = future.get();

              last_desire_ck_.found = response->found;
              last_desire_ck_.accepted = response->accepted;
              last_desire_ck_.arrived = true;
            }
        }
        catch(const rclcpp::exceptions::RCLError& rclerr)
        {
            //TODO fix this by avoiding to create a new client if a
            RCLCPP_ERROR(this->get_logger(), rclerr.what());
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Response error in " + serviceName);
        }

        //mtx_desire_req_lock_.unlock();
    }


    /*
      Utility methods to send desire request (ADD/DEL) to other agents within the action doWork method implementation
    */
    void sendDesireRequest(const string& agentRef, const Desire& desire, const UpdOperation& op, const bool& monitorFulfill)
    {
      //mtx_desire_req_lock_.lock();
      shared_ptr<thread> sendDesireReqThread = std::make_shared<thread>(
          bind(&BDIActionExecutor::updDesireRequestClientThread, this, agentRef, desire, op, monitorFulfill));
      sendDesireReqThread->detach();//wait for thread to terminate and get the response
      /*std::cout << "Desire request thread launched resp_arr=" << last_desire_req_resp_arrived_ << 
      "\\t req_acc=" << last_desire_req_accepted_ << std::endl;
      mtx_desire_req_lock_.lock();
      mtx_desire_req_lock_.unlock();
      std::cout << "Desire request thread finished resp_arr=" << last_desire_req_resp_arrived_ << 
      "\\t req_acc=" << last_desire_req_accepted_ << std::endl;*/
    }

    void updDesireRequestClientThread(const string& agentRef, const Desire& desire, const UpdOperation& op, const bool& monitorFulfill)
    {
      string serviceName = "/" + agentRef + "/" + 
          ((op==ADD)? "add" : "del") + 
          "_desire_srv";
      
      last_desire_upd_.desire = desire;
      last_desire_upd_.op = op;

      last_desire_upd_.arrived = false;
      last_desire_upd_.accepted = false;
      last_desire_upd_.performed = false;
      
      try{
            //check for service to be up
            rclcpp::Client<UpdDesireSet>::SharedPtr client_ = this->create_client<UpdDesireSet>(serviceName);
            if(!client_->wait_for_service(std::chrono::seconds(1)))
            {
                //service seems not even present, just return false
                RCLCPP_WARN(this->get_logger(), serviceName + " server does not appear to be up");
            }
            else
            {
              auto request = std::make_shared<UpdDesireSet::Request>();
              request->desire = desire;
              request->agent_group = agent_group_;

              auto future = client_->async_send_request(request);
              auto response = future.get();

              last_desire_upd_.performed = response->updated;
              last_desire_upd_.accepted = response->accepted;
              last_desire_upd_.arrived = true;
              
              if(response->accepted && response->updated && monitorFulfill)//desire request accepted and monitor fulfillment has been requested
                monitor(agentRef, desire);
            }
        }
        catch(const rclcpp::exceptions::RCLError& rclerr)
        {
            //TODO fix this by avoiding to create a new client if a
            RCLCPP_ERROR(this->get_logger(), rclerr.what());
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Response error in " + serviceName);
        }

        //mtx_desire_req_lock_.unlock();
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

    mutex mtx_desire_req_lock_;

    // last belief read request result
    CheckBeliefResult last_belief_ck_;
    // last belief upd request arrived/accepted/performed flag
    UpdBeliefResult last_belief_upd_;

    // last desire read request result
    CheckDesireResult last_desire_ck_;
    // last desire upd request arrived/accepted/performed flag
    UpdDesireResult last_desire_upd_;

    // progress of the current action execution
    float progress_;
};

#endif  // BDI_ACTION_EXECUTOR_H_
