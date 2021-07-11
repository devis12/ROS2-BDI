#include <cstdlib>
#include <optional>
#include <ctime>
#include <memory>
#include <vector>
#include <set>       

#include "plansys2_planner/PlannerClient.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"
#include "ros2_bdi_utils/PDDLBDIConstants.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_COMM_ERRORS 16
#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"
#define PARAM_PDDL_DOMAIN "pddl_test_domain" 
#define PARAM_PDDL_PROBLEM "pddl_test_problem" 

using std::string;
using std::vector;
using std::set;
using std::ios;
using std::ifstream;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;

using plansys2::PlannerClient;
using plansys2_msgs::msg::Plan;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::DesireSet;

typedef enum {STARTING, SCHEDULING, PAUSE} StateType;                

class Scheduler : public rclcpp::Node
{
public:
  Scheduler()
  : rclcpp::Node("scheduler"), state_(STARTING)
  {
    psys2_comm_errors_ = 0;
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);

    //define empty test PDDL domain and problem (loaded up from launch file)
    this->declare_parameter(PARAM_PDDL_DOMAIN, "(define (domain cleaner-domain) (:requirements :strips))");
    this->declare_parameter(PARAM_PDDL_PROBLEM, "(define (problem cleaner-problem) (:domain cleaner-domain))");
    planner_expert_up_ = false;
  }

  /*
    Init to call at the start, after construction method, to get the node actually started
    initialing planner client instance, 
    retrieving agent_id_ (thus namespace)
    defining work timer,
    belief set subscriber callback,
    desire set publisher,
    add/del desire subscribers callback
  */
  void init()
  { 
    //agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    // initializing planner client
    planner_client_ = std::make_shared<plansys2::PlannerClient>();

    //Init desire set
    desire_set_ = set<ManagedDesire>();

    //Desire set publisher
    desire_set_publisher_ = this->create_publisher<DesireSet>("desire_set", 10);

    //Desire to be added notification
    add_desire_subscriber_ = this->create_subscription<Desire>(
                "add_desire", 10,
                bind(&Scheduler::addDesireTopicCallBack, this, _1));

    //Desire to be removed notification
    del_desire_subscriber_ = this->create_subscription<Desire>(
                "del_desire", 10,
                bind(&Scheduler::delDesireTopicCallBack, this, _1));

    //belief_set_subscriber_ 
    belief_set_subscriber_ = this->create_subscription<BeliefSet>(
                "belief_set", 10,
                bind(&Scheduler::updatedBeliefSet, this, _1));

    //loop to be called regularly to perform work (publish belief_set_, sync with plansys2 problem_expert node...)
    do_work_timer_ = this->create_wall_timer(
        milliseconds(500),
        bind(&Scheduler::step, this));

    RCLCPP_INFO(this->get_logger(), "Scheduler node initialized");
  }
  
  /*
    Main loop of work called regularly through a wall timer
  */
  void step()
  {
    // check problem expert alive to init knowledge before spawning
    //if(psys2_comm_errors_ > MAX_COMM_ERRORS)
    //    rclcpp::shutdown();

    switch (state_) {
        
        case STARTING:
        {
            if(isPlannerActive()){
                planner_expert_up_ = true;
                psys2_comm_errors_ = 0;
                setState(SCHEDULING);
            }else{
                planner_expert_up_ = false;
                RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with PlanSys2 planner");
                psys2_comm_errors_++;
            }

            break;
        }

        case SCHEDULING:
        {    
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
       Publish the current desire set of the agent in agent_id_/desire_set topic
    */
    void publishDesireSet()
    {
        DesireSet dset_msg = BDIFilter::extractDesireSetMsg(desire_set_);
        desire_set_publisher_->publish(dset_msg);
    }

    /*
        Check if the plansys2 planner node is up by checking if it returns the correct plan if
        queried with a test domain and problem
    */
    bool isPlannerActive()
    {   
        bool isUp = false;
        pddl_domain_ = readTestPDDLDomain();
        //std::cout << pddl_domain_ << std::endl << std::endl << std::endl;
        pddl_problem_ = readTestPDDLProblem();
        //std::cout << pddl_problem_ << std::endl << std::endl << std::endl;
        std::optional<Plan> plan = planner_client_->getPlan(pddl_domain_, pddl_problem_);
        /*if(plan.has_value())
            RCLCPP_INFO(this->get_logger(), "Found plan: start with " + plan.value().items[0].action);*/
        return plan.has_value();
    }

    /*
        The belief set has been updated
    */
    void updatedBeliefSet(const BeliefSet::SharedPtr msg)
    {
        belief_set_ = BDIFilter::extractMGBeliefs(msg->value);
        //TODO has anything to be called after?
    }

    /*  
        Someone has publish a new desire to be fulfilled in the respective topic
    */
    void addDesireTopicCallBack(const Desire::SharedPtr msg)
    {
        //TODO implement logic
    }

    /*  
        Someone has publish a desire to be removed from the one to be fulfilled (if present)
        in the respective topic
    */
    void delDesireTopicCallBack(const Desire::SharedPtr msg)
    {
        //TODO implement logic
    }

    string readTestPDDLDomain()
    {
        return this->get_parameter(PARAM_PDDL_DOMAIN).as_string();
    }

    string readTestPDDLProblem()
    {
        return this->get_parameter(PARAM_PDDL_PROBLEM).as_string();
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
    std::shared_ptr<PlannerClient> planner_client_;
    // flag to denote if the problem expert node seems to correctly answer 
    bool planner_expert_up_;

    // belief set of the agent <agent_id_>
    set<ManagedBelief> belief_set_;

    // desire set of the agent <agent_id_>
    set<ManagedDesire> desire_set_;

    // string representing a pddl domain to supply to the plansys2 planner
    string pddl_domain_;
    // string representing a pddl problem to supply to the plansys2 planner
    string pddl_problem_;

    // desire set publishers/subscribers
    rclcpp::Subscription<Desire>::SharedPtr add_desire_subscriber_;//add desire notify on topic
    rclcpp::Subscription<Desire>::SharedPtr del_desire_subscriber_;//del desire notify on topic
    rclcpp::Publisher<DesireSet>::SharedPtr desire_set_publisher_;//desire set publisher

    // belief set subscriber
    rclcpp::Subscription<BeliefSet>::SharedPtr belief_set_subscriber_;//desire set publisher
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Scheduler>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
