#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_utils/PDDLBDIConverter.h"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp/rclcpp.hpp"


#define MAX_COMM_ERRORS 16


using std::string;
using std::vector;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;

using plansys2::ProblemExpertClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;
using plansys2_msgs::srv::GetProblemInstances;
using std_msgs::msg::Empty;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;

typedef enum {STARTING, SYNC, PAUSE} StateType;

class BeliefManager : public rclcpp::Node
{
public:
  BeliefManager()
  : rclcpp::Node("belief_manager"), state_(STARTING)
  {
    psys2_comm_errors_ = 0;
    this->declare_parameter("agent_id", "agent0");
    problem_expert_up_ = false;
  }

  /*
    Init to call at the start, after construction method, to get the node actually started
    initialing problem_expert_ instance, 
    retrieving agent_id_ (thus namespace)
    defining work timer,
    belief set publishers/subscribers callback,
    problem_expert_update notification
  */
  void init()
  { 
    agent_id_ = this->get_parameter("agent_id").as_string();
    problem_expert_ = std::make_shared<ProblemExpertClient>();

    //Init belief set
    belief_set_ = BeliefSet();
    belief_set_.value = vector<Belief>();

    //Belief set publisher
    belief_set_publisher_ = this->create_publisher<BeliefSet>("belief_set", 10);
    //Belief to be added notification
    add_belief_subscriber_ = this->create_subscription<Belief>(
                "add_belief", 10,
                bind(&BeliefManager::addBelief, this, _1));
    //Belief to be removed notification
    del_belief_subscriber_ = this->create_subscription<Belief>(
                "del_belief", 10,
                bind(&BeliefManager::delBelief, this, _1));

    //problem_expert update subscriber
    updated_problem_subscriber_ = this->create_subscription<Empty>(
                "problem_expert/update_notify", 10,
                bind(&BeliefManager::updatedPDDLProblem, this, _1));

    //loop to be called regularly to perform work (publish belief_set_, sync with plansys2 problem_expert node...)
    do_work_timer_ = this->create_wall_timer(
        milliseconds(4),
        bind(&BeliefManager::step, this));

    RCLCPP_INFO(this->get_logger(), "Belief manager node initialized");
  }
  
  /*
    Main loop of work called regularly through a wall timer
  */
  void step()
  {
    // check problem expert alive to init knowledge before spawning
    if(psys2_comm_errors_ > MAX_COMM_ERRORS)
        rclcpp::shutdown();

    switch (state_) {
        
        case STARTING:
        {
            if(isProblemExpertActive()){
                problem_expert_up_ = true;
                psys2_comm_errors_ = 0;
                setState(SYNC);
            }else{
                problem_expert_up_ = false;
                RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with problem expert");
                psys2_comm_errors_++;
            }

            break;
        }

        case SYNC:
        {    
            publishBeliefSet();
        }

        case PAUSE:
        {   
            //RCLCPP_INFO(this->get_logger(), "Not the moment to ask for new cleaning tasks yet");    
            break;
        }
        default:
            break;
    }
  }

private:
    /*  
        Change internal state of the node
    */
    void setState(StateType state)
    {
        state_ = state;
    }


    /*
        Check if the plansys2 problem expert node is up by checking for the presence of one of its services 
        (specifically problem_expert/get_problem_instances) and testing then out by adding and removing instances
        through the cpp api calls
    */
    bool isProblemExpertActive()
    {   
        //check for service to be up
        rclcpp::Client<GetProblemInstances>::SharedPtr client_ = 
            this->create_client<GetProblemInstances>("problem_expert/get_problem_instances");
        if(!client_->wait_for_service(std::chrono::seconds(1))){
            //service seems not even present, just return false
            RCLCPP_WARN(this->get_logger(), "problem_expert/get_problem_instances server does not appear to be up");
            return false;
        }

        //service listed, but is the node actually in an active state? test it with the cpp api (add+remove test instance)
        auto test_name = "bm" + agent_id_ + "_wyp";
        bool isUp = problem_expert_->addInstance(Instance{test_name, "waypoint"});//check if up with test instance
        isUp = isUp && problem_expert_->removeInstance(Instance{test_name, "waypoint"});
        
        return isUp;
    }

    void publishBeliefSet()
    {
        belief_set_publisher_->publish(belief_set_);
    }

    void updatedPDDLProblem(const Empty::SharedPtr msg)
    {
        if(problem_expert_up_ || isProblemExpertActive())
        {
            //vector<Instance> instances = problem_expert_->getInstances();
            vector<Belief> predicates = PDDLBDIConverter::convertPDDLPredicates(problem_expert_->getPredicates());
            vector<Belief> functions = PDDLBDIConverter::convertPDDLFunctions(problem_expert_->getFunctions());

            //TODO implement logic to update BeliefSet
        }
    }

    void addBelief(const Belief::SharedPtr msg)
    {
        //TODO not implemented yet
    }

    void delBelief(const Belief::SharedPtr msg)
    {
        //TODO not implemented yet
    }
    

    // internal state of the node
    StateType state_;
    
    // agent id that defines the namespace in which the node operates
    string agent_id_;
    // callback to perform main loop of work regularly
    rclcpp::TimerBase::SharedPtr do_work_timer_;

    // counter of communication errors with plansys2
    int psys2_comm_errors_;
    // problem expert instance to call the problem expert api
    std::shared_ptr<ProblemExpertClient> problem_expert_;
    // flag to denote if the problem expert node seems to correctly answer 
    bool problem_expert_up_;

    // belief set of the agent <agent_id_>
    BeliefSet belief_set_;

    // belief set publishers/subscribers
    rclcpp::Subscription<Belief>::SharedPtr add_belief_subscriber_;//add belief notify on topic
    rclcpp::Subscription<Belief>::SharedPtr del_belief_subscriber_;//del belief notify on topic
    rclcpp::Publisher<BeliefSet>::SharedPtr belief_set_publisher_;//belief set publisher

    // plansys2 problem expert notification for updates
    rclcpp::Subscription<Empty>::SharedPtr updated_problem_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BeliefManager>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
