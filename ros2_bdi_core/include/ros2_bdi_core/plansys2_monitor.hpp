#ifndef PLANSYS2_MONITOR_H_
#define PLANSYS2_MONITOR_H_

#include <string>
#include <memory>

#include "ros2_bdi_core/support/plansys2_monitor_client.hpp"

#include "ros2_bdi_interfaces/msg/plan_sys2_state.hpp"

#include "rclcpp/rclcpp.hpp"

class PlanSys2Monitor : public rclcpp::Node
{
    public:

        /* Constructor method */
        PlanSys2Monitor();

        /*
            Init to call at the start, after construction method, to get the node actually started
            init timer to regularly check plansys2 node state
        */
        void init();
        
        /*
            Main loop of work called regularly through a wall timer
        */
        void checkPlanSys2State();

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
        void checkPsys2NodeActive(const std::string& psys2NodeName);

        
        // agent id that defines the namespace in which the node operates
        std::string agent_id_;
        // callback to perform main loop of work regularly
        rclcpp::TimerBase::SharedPtr do_work_timer_;
        // work timer interval in ms
        int work_timer_interval_;

        // psys2 nodes active flags
        ros2_bdi_interfaces::msg::PlanSys2State psys2_active_;

        // comm errors psys2
        int psys2_comm_errors_;
        
        // PlanSys2 Monitor Client supporting nodes & clients for calling the {psys2_node}/get_state services
        std::shared_ptr<PlanSys2MonitorClient> psys2_monitor_client_;
        
        // PlanSys2 state publisher
        rclcpp::Publisher<ros2_bdi_interfaces::msg::PlanSys2State>::SharedPtr psys2_state_publisher_;

}; // PlanSys2Monitor class prototype

#endif // PLANSYS2_MONITOR_H_