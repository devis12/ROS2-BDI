// header file for PlanSys2 Monitor node
#include "ros2_bdi_core/plansys_monitor.hpp"
// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for PlanSys2 Monitor node
#include "ros2_bdi_core/params/plansys_monitor_params.hpp"
// Scheduler params
#include "ros2_bdi_core/params/scheduler_params.hpp"

using std::string;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;

using ros2_bdi_interfaces::msg::PlanningSystemState;


PlanSysMonitor::PlanSysMonitor() : rclcpp::Node(PSYS2_MONITOR_NODE_NAME)
{
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);
    this->declare_parameter(PARAM_PLANNING_MODE, PLANNING_MODE_OFFLINE);

    sel_planning_mode_ = this->get_parameter(PARAM_PLANNING_MODE).as_string() == PLANNING_MODE_OFFLINE? OFFLINE : ONLINE;
    this->undeclare_parameter(PARAM_PLANNING_MODE);

    psys_monitor_client_ = std::make_shared<PlanSysMonitorClient>(PSYS2_MONITOR_NODE_NAME + string("_caller_"), sel_planning_mode_);
}

/*
    Init to call at the start, after construction method, to get the node actually started
    init timer to regularly check plansys2 node state
*/
void PlanSysMonitor::init()
{ 
    // agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    // PlanSys2 state monitor publisher
    psys_state_publisher_ = this->create_publisher<PlanningSystemState>(PSYS2_STATE_TOPIC, 10);

    // set at start work timer interval at minimum (so it checks very frequently)
    // if all up & active, it'll grow, checking the services less frequently
    work_timer_interval_ = TIMER_MIN;
    //loop to be called regularly to perform work (check plansys2 node states)
    do_work_timer_ = this->create_wall_timer(
            milliseconds(work_timer_interval_),
            bind(&PlanSysMonitor::checkPlanningSystemState, this));

    psys2_comm_errors_ = 0;

    // init flag values to false
    psys_active_ = PlanningSystemState{};
    psys_active_.domain_expert_active = false;
    psys_active_.problem_expert_active = false;
    psys_active_.offline_planner_active = false;
    psys_active_.online_planner_active = false;
    psys_active_.executor_active = false;


    RCLCPP_INFO(this->get_logger(), "PlanSys2 Monitor node initialized");
}

/*
Main loop of work called regularly through a wall timer
*/
void PlanSysMonitor::checkPlanningSystemState()
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

    checkPsysNodeActive(PSYS2_DOM_EXPERT);// check if domain expert is up & active
    checkPsysNodeActive(PSYS2_PROB_EXPERT);// check if problem expert is up & active

    if(sel_planning_mode_ == OFFLINE)
        checkPsysNodeActive(PSYS2_PLANNER);// check if psys2 planner is up & active
    else if(sel_planning_mode_ == ONLINE)
        checkPsysNodeActive(JAVAFF_PLANNER);// check if javaff online planner is up & active

    checkPsysNodeActive(PSYS2_EXECUTOR);// check if problem expert is up & active

    //publish current state
    psys_state_publisher_->publish(psys_active_);
}

/*
    Init work timer with current timer interval (which can change over time)
*/
void PlanSysMonitor::resetWorkTimer()
{
    if(!do_work_timer_->is_canceled())
        do_work_timer_->cancel();

    do_work_timer_ = this->create_wall_timer(
        milliseconds(work_timer_interval_),
        bind(&PlanSysMonitor::checkPlanningSystemState, this));
}

/*
    returns true iff the flags notifying all psys2 nodes are active
*/
bool PlanSysMonitor::allActive()
{
    return psys_active_.domain_expert_active && psys_active_.problem_expert_active &&
            psys_active_.executor_active && (psys_active_.offline_planner_active || psys_active_.online_planner_active);        
}

/*
    Activate supporting client node to check active status of plansys2 node (planner, domain_expert, problem_expert)

    Use result to update corresponding flag
*/
void PlanSysMonitor::checkPsysNodeActive(const string& psysNodeName)
{
    bool activeResult = psys_monitor_client_->isPsysNodeActive(psysNodeName);

    //assign active flag to respective boolean value representing the active state of the PlanSys2 node
    if(psysNodeName == PSYS2_DOM_EXPERT)
        psys_active_.domain_expert_active = activeResult;
    else if(psysNodeName == PSYS2_PROB_EXPERT)
        psys_active_.problem_expert_active = activeResult;
    else if(psysNodeName == PSYS2_PLANNER)
        psys_active_.offline_planner_active = activeResult;
    else if(psysNodeName == JAVAFF_PLANNER)
        psys_active_.online_planner_active = activeResult;
    else if(psysNodeName == PSYS2_EXECUTOR)
        psys_active_.executor_active = activeResult;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanSysMonitor>();
  std::this_thread::sleep_for(std::chrono::seconds(1));//WAIT PSYS2 TO BOOT

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
