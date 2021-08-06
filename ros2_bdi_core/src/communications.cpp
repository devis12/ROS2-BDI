#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>
#include <thread>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_interfaces/srv/add_belief.hpp"
#include "ros2_bdi_interfaces/srv/del_belief.hpp"
#include "ros2_bdi_interfaces/srv/add_desire.hpp"
#include "ros2_bdi_interfaces/srv/del_desire.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"

#include "rclcpp/rclcpp.hpp"

#define PARAM_AGENT_ID "agent_id"
#define PARAM_AGENT_GROUP_ID "agent_group"
#define PARAM_BELIEF_ACCEPT "accept_beliefs_from"
#define PARAM_DESIRE_ACCEPT "accept_desires_from"
#define PARAM_DESIRE_MAX_PRIORITIES "accept_desires_max_priorities"
#define PARAM_DEBUG "debug"

using std::string;
using std::vector;
using std::set;
using std::thread;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::placeholders::_2;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::DesireSet;
using ros2_bdi_interfaces::srv::AddBelief;
using ros2_bdi_interfaces::srv::DelBelief;
using ros2_bdi_interfaces::srv::AddDesire;
using ros2_bdi_interfaces::srv::DelDesire;

typedef enum {BELIEF, DESIRE} RequestObjType;   

class CommunicationManager : public rclcpp::Node
{
public:
  CommunicationManager()
  : rclcpp::Node("communication_manager")
  {
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);
    this->declare_parameter(PARAM_BELIEF_ACCEPT, vector<string>());
    this->declare_parameter(PARAM_DESIRE_ACCEPT, vector<string>());
    this->declare_parameter(PARAM_DESIRE_MAX_PRIORITIES, vector<double>());
  }

  /*
    Init to call at the start, after construction method, to get the node actually started
  */
  void init()
  { 
    // agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
    qos_keep_all.keep_all();

    //register to belief set updates to have the mirroring of the last published version of it
    belief_set_subscriber_ = this->create_subscription<BeliefSet>(
                "belief_set", qos_keep_all,
                bind(&CommunicationManager::updatedBeliefSet, this, _1));
    
    //register to desire set updates to have the mirroring of the last published version of it
    desire_set_subscriber_ = this->create_subscription<DesireSet>(
                "desire_set", qos_keep_all,
                bind(&CommunicationManager::updatedDesireSet, this, _1));

    // init server for handling add belief requests from other agents
    add_belief_server_ = this->create_service<AddBelief>("add_belief_srv", 
        bind(&CommunicationManager::handleAddBeliefRequest, this, _1, _2));
      
     // init server for handling del belief requests from other agents
    del_belief_server_ = this->create_service<DelBelief>("del_belief_srv", 
        bind(&CommunicationManager::handleDelBeliefRequest, this, _1, _2));
    
    // add belief publisher -> to publish on the topic and alter the belief set when the request can go through
    add_belief_publisher_ = this->create_publisher<Belief>("add_belief", 10);
    // del belief publisher -> to publish on the topic and alter the belief set when the request can go through
    del_belief_publisher_ = this->create_publisher<Belief>("del_belief", 10);


    // init server for handling add belief requests from other agents
    add_desire_server_ = this->create_service<AddDesire>("add_desire_srv", 
        bind(&CommunicationManager::handleAddDesireRequest, this, _1, _2));
      
     // init server for handling del belief requests from other agents
    del_desire_server_ = this->create_service<DelDesire>("del_desire_srv", 
        bind(&CommunicationManager::handleDelDesireRequest, this, _1, _2));

    // add desire publisher -> to publish on the topic and alter the desire set when the request can go through
    add_desire_publisher_ = this->create_publisher<Desire>("add_desire", 10);
    // del desire publisher -> to publish on the topic and alter the desire set when the request can go through
    del_desire_publisher_ = this->create_publisher<Desire>("del_desire", 10);

    RCLCPP_INFO(this->get_logger(), "Communications Manager node initialized");
  }
  

private:

    /*
      Return true if the request agent's group name is among the accepted ones wrt.
      either belief or desire modify acceptance 
    */
    bool isAcceptableRequest(const string& requestingAgentGroup, const RequestObjType& requestObjType)
    {
      vector<string> acceptedGroups;
      switch(requestObjType)
      {
        case BELIEF:
          acceptedGroups = this->get_parameter(PARAM_BELIEF_ACCEPT).as_string_array();

         case DESIRE:
          acceptedGroups = this->get_parameter(PARAM_DESIRE_ACCEPT).as_string_array();
      }

      for(string accepted : acceptedGroups)
        if(accepted == requestingAgentGroup)
          return true;// found among accepted ones

      return false;// not found among accepted ones
    }

    /*
      Return value of the max accepted priority for add desire requests coming from a requesting agent group
      Return negative value if not present
    */
    float getMaxAcceptedPriority(const string& requestingAgentGroup)
    {
      int indexAccepted = -1;
      float maxAcceptedPriority = -1.0f;//init to negative value

      // find index of accepted group in accept_desires_from array string paramters
      vector<string> acceptedGroups = this->get_parameter(PARAM_DESIRE_ACCEPT).as_string_array();
      for(int i = 0; indexAccepted == -1 && i <  acceptedGroups.size(); i++)
        if(acceptedGroups[i] == requestingAgentGroup)
          indexAccepted = i;

      if(indexAccepted > 0)//if found
      {
        // retrieve correspondent priority from accept_desires_max_priorities double array
        vector<double> acceptedPriorities = this->get_parameter(PARAM_DESIRE_MAX_PRIORITIES).as_double_array();
        
        if(indexAccepted < acceptedPriorities.size())
          maxAcceptedPriority = std::max(0.0f, std::min(1.0f, (float)acceptedPriorities[indexAccepted]));//priority has to be between 0 and 1
        
        else
          maxAcceptedPriority = 0.0f; //bad formatted accept_desires_max_priorities array, but do not refuse just put 0.0 as priority of the desire
      }

      return maxAcceptedPriority;
    }

    /*
        The desire set has been updated
    */
    void updatedDesireSet(const DesireSet::SharedPtr msg)
    {
        desire_set_ = BDIFilter::extractMGDesires(msg->value);
    }
    
    /*
        The belief set has been updated
    */
    void updatedBeliefSet(const BeliefSet::SharedPtr msg)
    {
        belief_set_ = BDIFilter::extractMGBeliefs(msg->value);
    }


    /*  
        Add Belief Request service handler
    */
    void handleAddBeliefRequest(const AddBelief::Request::SharedPtr request,
        const AddBelief::Response::SharedPtr response)
    {
      //see if the requesting agent belongs to a group which is entitled to this kind of requests
      if(!isAcceptableRequest(request->agent_group, BELIEF))
        response->accepted = false;

      else
      {
        add_belief_publisher_->publish(request->belief);
        response->accepted = true;// [TODO] check if the modification has gone forward
      }
    }

    /*  
        Del Belief Request service handler        
    */
    void handleDelBeliefRequest(const DelBelief::Request::SharedPtr request,
        const DelBelief::Response::SharedPtr response)
    {
      //see if the requesting agent belongs to a group which is entitled to this kind of requests
      if(!isAcceptableRequest(request->agent_group, BELIEF))
        response->accepted = false;

      else
      {
        del_belief_publisher_->publish(request->belief);
        response->accepted = true;// [TODO] check if the modification has gone forward
      }
    }


    /*  
        Add Desire Request service handler        
    */
    void handleAddDesireRequest(const AddDesire::Request::SharedPtr request,
        const AddDesire::Response::SharedPtr response)
    {
      //see if the requesting agent belongs to a group which is entitled to this kind of requests
      if(!isAcceptableRequest(request->agent_group, DESIRE))
        response->accepted = false;

      else
      {
        float maxAcceptedPriority = getMaxAcceptedPriority(request->agent_group);
        if(maxAcceptedPriority >= 0)
        {
          // set at most the desire priority to the fixed upper threshold
          request->desire.priority = std::min(request->desire.priority, maxAcceptedPriority); 
          add_desire_publisher_->publish(request->desire);
          response->accepted = true;// [TODO] check if the modification has gone forward
        }
        else
          response->accepted = false;// max priority for given agent's requesting group is negative -> not accepted
        
      }
    }

    /*  
         Del Desire Request service handler       
    */
    void handleDelDesireRequest(const DelDesire::Request::SharedPtr request,
        const DelDesire::Response::SharedPtr response)
    {
      //see if the requesting agent belongs to a group which is entitled to this kind of requests
      if(!isAcceptableRequest(request->agent_group, DESIRE))
        response->accepted = false;

      else
      {
        del_desire_publisher_->publish(request->desire);
        response->accepted = true;// [TODO] check if the modification has gone forward
      }
    }
    
    // agent id that defines the namespace in which the node operates
    string agent_id_;

    // mirroring of the current state of the belief set
    set<ManagedBelief> belief_set_;
    // belief set update subscription
    rclcpp::Subscription<BeliefSet>::SharedPtr belief_set_subscriber_;

    // mirroring of the current state of the desire set
    set<ManagedDesire> desire_set_;
    // desire set update subscription
    rclcpp::Subscription<DesireSet>::SharedPtr desire_set_subscriber_;

    // handle add belief requests from other agents
    rclcpp::Service<AddBelief>::SharedPtr add_belief_server_;
    // handle del belief requests from other agents
    rclcpp::Service<DelBelief>::SharedPtr del_belief_server_;
    //add_belief publisher
    rclcpp::Publisher<Belief>::SharedPtr add_belief_publisher_;
    //del_belief publisher
    rclcpp::Publisher<Belief>::SharedPtr del_belief_publisher_;

    // handle add desire requests from other agents
    rclcpp::Service<AddDesire>::SharedPtr add_desire_server_;
    // handle del desire requests from other agents
    rclcpp::Service<DelDesire>::SharedPtr del_desire_server_;
    //add_desire publisher
    rclcpp::Publisher<Desire>::SharedPtr add_desire_publisher_;
    //del_desire publisher
    rclcpp::Publisher<Desire>::SharedPtr del_desire_publisher_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommunicationManager>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
