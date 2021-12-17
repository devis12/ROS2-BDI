#include "ros2_bdi_core/plansys2_monitor.hpp"

#include <thread>

/* Parameters affecting internal logic (recompiling required) */
#define MAX_COMM_ERRORS 16
#define TIMER_MIN 250
#define TIMER_MAX 2000

/* ROS2 Parameter names for PlanSys2Monitor node */
#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"

using std::string;
using std::vector;
using std::thread;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;

using ros2_bdi_interfaces::msg::PlanSys2State;

PlanSys2Monitor::PlanSys2Monitor() : rclcpp::Node("plansys2_monitor")
{
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);
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

    psys2_domain_expert_active_ = false;
    psys2_problem_expert_active_ = false;
    psys2_planner_active_ = false;
    psys2_executor_active_ = false;

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

    checkPsys2NodeActive("domain_expert");// check if domain expert is up & active
    checkPsys2NodeActive("problem_expert");// check if problem expert is up & active
    checkPsys2NodeActive("planner");// check if problem expert is up & active
    checkPsys2NodeActive("executor");// check if problem expert is up & active

    //publish current state
    PlanSys2State psys2State = buildPlanSys2StateMsg();
    psys2_state_publisher_->publish(psys2State);
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
    return psys2_domain_expert_active_ && psys2_problem_expert_active_ &&
            psys2_executor_active_ && psys2_planner_active_; 
            
}

/*
    Build PlanSys2 state msg wrt the current state of the respective boolean flags
*/
PlanSys2State PlanSys2Monitor::buildPlanSys2StateMsg()
{
    PlanSys2State msg = PlanSys2State();
    msg.domain_expert_active = psys2_domain_expert_active_;
    msg.problem_expert_active = psys2_problem_expert_active_;
    msg.planner_active = psys2_planner_active_;
    msg.executor_active = psys2_executor_active_;
    return msg;
}

/*
    Activate thread to check active state of plansys2 node (planner, domain_expert, problem_expert)
*/
void PlanSys2Monitor::checkPsys2NodeActive(const string& psys2NodeName)
{
    if(psys2NodeName == "domain_expert")
    {
        chk_domain_expert_thread_ = std::make_shared<thread>(bind(&PlanSys2Monitor::checkPsys2NodeActiveThread, this, psys2NodeName));
        chk_domain_expert_thread_->detach();
    }
    else if(psys2NodeName == "problem_expert")
    {
        chk_problem_expert_thread_ = std::make_shared<thread>(bind(&PlanSys2Monitor::checkPsys2NodeActiveThread, this, psys2NodeName));
        chk_problem_expert_thread_->detach();
    }
    else if(psys2NodeName == "planner")
    {
        chk_planner_thread_ = std::make_shared<thread>(bind(&PlanSys2Monitor::checkPsys2NodeActiveThread, this, psys2NodeName));
        chk_planner_thread_->detach();
    }
    else if(psys2NodeName == "executor")
    {
        chk_executor_thread_ = std::make_shared<thread>(bind(&PlanSys2Monitor::checkPsys2NodeActiveThread, this, psys2NodeName));
        chk_executor_thread_->detach();
    }
}

/*
    Check with a srv call to its respective get_state service if the plansys2 planner/domain_expert/problem_expert is active
    @psys2NodeName is equal to one of the plansys2 nodes you want to check if active through get_state srv
*/
void PlanSys2Monitor::checkPsys2NodeActiveThread(const string& psys2NodeName)
{
    RCLCPP_DEBUG(this->get_logger(), "Checking active state of " + psys2NodeName);
    bool active = false;
    try{
        //check for service to be up
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_ = 
            this->create_client<lifecycle_msgs::srv::GetState>(psys2NodeName + "/get_state");
        if(!client_->wait_for_service(std::chrono::seconds(1))){
            //service seems not even present, just return false
            RCLCPP_WARN(this->get_logger(), psys2NodeName + "/get_state server does not appear to be up");
            active = false;
        }
        else
        {
            // service up -> query it to know PlanSys2 node active state
            auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
            auto future = client_->async_send_request(request);
            auto response = future.get();
            active = response->current_state.id == 3 && response->current_state.label == "active";
        }
        

    }
    catch(const rclcpp::exceptions::RCLError& rclerr)
    {
        active = false;
        RCLCPP_ERROR(this->get_logger(), rclerr.what());
    }
    catch(const std::exception &e)
    {
        active = false;
        RCLCPP_ERROR(this->get_logger(), "Response error in " + psys2NodeName + "/get_state");
    }

    //assign active flag to respective boolean value representing the active state of the PlanSys2 node
    if(psys2NodeName == "domain_expert")
        psys2_domain_expert_active_ = active;
    else if(psys2NodeName == "problem_expert")
        psys2_problem_expert_active_ = active;
    else if(psys2NodeName == "planner")
        psys2_planner_active_ = active;
    else if(psys2NodeName == "executor")
        psys2_executor_active_ = active;
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
