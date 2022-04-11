// header file for PlanSys2 Monitor node
#include "ros2_bdi_core/plansys2_monitor.hpp"
// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for PlanSys2 Monitor node
#include "ros2_bdi_core/params/plansys2_monitor_params.hpp"

using std::string;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;

using ros2_bdi_interfaces::msg::PlanSys2State;


PlanSys2Monitor::PlanSys2Monitor() : rclcpp::Node(PSYS2_MONITOR_NODE_NAME)
{
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);

    psys2_monitor_client_ = std::make_shared<PlanSys2MonitorClient>(PSYS2_MONITOR_NODE_NAME + string("_caller_"));
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
    psys2_state_publisher_ = this->create_publisher<PlanSys2State>(PSYS2_STATE_TOPIC, 10);

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


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanSys2Monitor>();
  std::this_thread::sleep_for(std::chrono::seconds(3));//WAIT PSYS2 TO BOOT

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
