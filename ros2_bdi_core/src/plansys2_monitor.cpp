#include "ros2_bdi_core/plansys2_monitor.hpp"

#include <thread>

/* Parameters affecting internal logic (recompiling required) */
#define MAX_COMM_ERRORS 16
#define TIMER_MIN 250
#define TIMER_MAX 2000

#define PSYS2NODES 4

// psys2 nodes' names
#define PSYS2_DOM_EXPERT "domain_expert"
#define PSYS2_PROB_EXPERT "problem_expert"
#define PSYS2_PLANNER "planner"
#define PSYS2_EXECUTOR "executor"

// index for the node and client callers
#define PSYS2_DOM_EXPERT_I 0
#define PSYS2_PROB_EXPERT_I 1
#define PSYS2_PLANNER_I 2
#define PSYS2_EXECUTOR_I 3

//seconds to wait before giving up on performing any request (service does not appear to be up)
#define WAIT_SRV_UP 1   

//seconds to wait before giving up on waiting for the response
#define WAIT_RESPONSE_TIMEOUT 1

/* ROS2 Parameter names for PlanSys2Monitor node */
#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"

using std::string;
using std::vector;
// using std::thread;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;

using lifecycle_msgs::srv::GetState;

using ros2_bdi_interfaces::msg::PlanSys2State;


PlanSys2Monitor::PlanSys2Monitor() : rclcpp::Node("plansys2_monitor")
{
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);

    psys2_monitor_client_ = std::make_shared<PlanSys2MonitorClient>();
}

/*
    Init to call at the start, after construction method, to get the node actually started
    init timer to regularly check plansys2 node state
*/
void PlanSys2Monitor::init()
{ 
    // agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    // PlanSys2 state monitor publisher
    psys2_state_publisher_ = this->create_publisher<PlanSys2State>("plansys2_state", 10);

    // set at start work timer interval at minimum (so it checks very frequently)
    // if all up & active, it'll grow, checking the services less frequently
    work_timer_interval_ = TIMER_MIN;
    //loop to be called regularly to perform work (check plansys2 node states)
    do_work_timer_ = this->create_wall_timer(
            milliseconds(work_timer_interval_),
            bind(&PlanSys2Monitor::checkPlanSys2State, this));

    psys2_comm_errors_ = 0;

    // init flag values to false
    psys2_active_ = PlanSys2State{};
    psys2_active_.domain_expert_active = false;
    psys2_active_.problem_expert_active = false;
    psys2_active_.planner_active = false;
    psys2_active_.executor_active = false;

    RCLCPP_INFO(this->get_logger(), "PlanSys2 Monitor node initialized");
}

/*
Main loop of work called regularly through a wall timer
*/
void PlanSys2Monitor::checkPlanSys2State()
{
    //if psys2 appears crashed, crash too
    if(psys2_comm_errors_ > MAX_COMM_ERRORS)
        rclcpp::shutdown();

    bool psys2NodesActive = allActive();

    if(psys2NodesActive && work_timer_interval_ < TIMER_MAX)
    {
        //all psys2 nodes are active & the work timer interval can still be enlarge
        psys2_comm_errors_ = 0;
        work_timer_interval_ += TIMER_MIN;
        resetWorkTimer();

    }else if(!psys2NodesActive && work_timer_interval_ > TIMER_MIN){
        psys2_comm_errors_++;
        //some psys2 nodes are not active & the work timer interval has to be put down to the minimum
        work_timer_interval_ = TIMER_MIN;
        resetWorkTimer();
    }

    checkPsys2NodeActive(PSYS2_DOM_EXPERT);// check if domain expert is up & active
    checkPsys2NodeActive(PSYS2_PROB_EXPERT);// check if problem expert is up & active
    checkPsys2NodeActive(PSYS2_PLANNER);// check if problem expert is up & active
    checkPsys2NodeActive(PSYS2_EXECUTOR);// check if problem expert is up & active

    //publish current state
    psys2_state_publisher_->publish(psys2_active_);
}

/*
    Init work timer with current timer interval (which can change over time)
*/
void PlanSys2Monitor::resetWorkTimer()
{
    if(!do_work_timer_->is_canceled())
        do_work_timer_->cancel();

    do_work_timer_ = this->create_wall_timer(
        milliseconds(work_timer_interval_),
        bind(&PlanSys2Monitor::checkPlanSys2State, this));
}

/*
    returns true iff the flags notifying all psys2 nodes are active
*/
bool PlanSys2Monitor::allActive()
{
    return psys2_active_.domain_expert_active && psys2_active_.problem_expert_active &&
            psys2_active_.executor_active && psys2_active_.planner_active;        
}

/*
    Activate supporting client node to check active status of plansys2 node (planner, domain_expert, problem_expert)

    Use result to update corresponding flag
*/
void PlanSys2Monitor::checkPsys2NodeActive(const string& psys2NodeName)
{
    bool activeResult = psys2_monitor_client_->isPsys2NodeActive(psys2NodeName);

    //assign active flag to respective boolean value representing the active state of the PlanSys2 node
    if(psys2NodeName == PSYS2_DOM_EXPERT)
        psys2_active_.domain_expert_active = activeResult;
    else if(psys2NodeName == PSYS2_PROB_EXPERT)
        psys2_active_.problem_expert_active = activeResult;
    else if(psys2NodeName == PSYS2_PLANNER)
        psys2_active_.planner_active = activeResult;
    else if(psys2NodeName == PSYS2_EXECUTOR)
        psys2_active_.executor_active = activeResult;
}


PlanSys2MonitorClient::PlanSys2MonitorClient()
{   
    /*Init caller nodes*/
    caller_nodes_ = vector<rclcpp::Node::SharedPtr>();
    for(int i=0; i<PSYS2NODES; i++)
        caller_nodes_.push_back(rclcpp::Node::make_shared("plansys2_state_caller_"+i));
    
    /*Init caller clients*/
    caller_clients_ = vector<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>();
    string domain_expert_name = PSYS2_DOM_EXPERT;
    caller_clients_.push_back(caller_nodes_[PSYS2_DOM_EXPERT_I]->create_client<lifecycle_msgs::srv::GetState>(domain_expert_name + "/get_state"));  
    string problem_expert_name = PSYS2_PROB_EXPERT;
    caller_clients_.push_back(caller_nodes_[PSYS2_PROB_EXPERT_I]->create_client<lifecycle_msgs::srv::GetState>(problem_expert_name + "/get_state"));  
    string planner_name = PSYS2_PLANNER;
    caller_clients_.push_back(caller_nodes_[PSYS2_PLANNER_I]->create_client<lifecycle_msgs::srv::GetState>(planner_name + "/get_state"));  
    string executor_name = PSYS2_EXECUTOR;
    caller_clients_.push_back(caller_nodes_[PSYS2_EXECUTOR_I]->create_client<lifecycle_msgs::srv::GetState>(executor_name + "/get_state"));  
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


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanSys2Monitor>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
