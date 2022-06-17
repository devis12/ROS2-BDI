#ifndef PLANSYS_MONITOR_H_
#define PLANSYS_MONITOR_H_

#include <string>
#include <memory>

#include "ros2_bdi_core/support/plansys_monitor_client.hpp"
#include "ros2_bdi_core/support/planning_mode.hpp"

#include "ros2_bdi_interfaces/msg/planning_system_state.hpp"

#include "rclcpp/rclcpp.hpp"

class PlanSysMonitor : public rclcpp::Node
{
    public:

        /* Constructor method */
        PlanSysMonitor();

        /*
            Init to call at the start, after construction method, to get the node actually started
            init timer to regularly check plansys2 node state
        */
        void init();
        
        /*
            Main loop of work called regularly through a wall timer
        */
        void checkPlanningSystemState();

    private:

        /*
            Init work timer with current timer interval (which can change over time)
        */
        void resetWorkTimer();

        /*
            returns true iff the flags notifying all psys2 nodes are active
        */
        bool allActive();

        /*
            Activate thread to check active state of plansys2 node (planner, domain_expert, problem_expert)
        */
        void checkPsysNodeActive(const std::string& psysNodeName);

        // Selected planning mode
        PlanningMode sel_planning_mode_;
        // agent id that defines the namespace in which the node operates
        std::string agent_id_;
        // callback to perform main loop of work regularly
        rclcpp::TimerBase::SharedPtr do_work_timer_;
        // work timer interval in ms
        int work_timer_interval_;

        // psys2 nodes active flags
        ros2_bdi_interfaces::msg::PlanningSystemState psys_active_;

        // comm errors psys2
        int psys2_comm_errors_;
        
        // PlanSys2 Monitor Client supporting nodes & clients for calling the {psys2_node}/get_state services
        std::shared_ptr<PlanSysMonitorClient> psys_monitor_client_;
        
        // PlanSys2 state publisher
        rclcpp::Publisher<ros2_bdi_interfaces::msg::PlanningSystemState>::SharedPtr psys_state_publisher_;

}; // PlanSys2Monitor class prototype

#endif // PLANSYS_MONITOR_H_