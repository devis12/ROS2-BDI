#ifndef PLANSYS_MONITOR_H_
#define PLANSYS_MONITOR_H_

#include <string>
#include <memory>
#include <map>

#include "ros2_bdi_interfaces/msg/lifecycle_status.hpp"
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

        /*Build updated ros2_bdi_interfaces::msg::LifecycleStatus msg*/
        ros2_bdi_interfaces::msg::LifecycleStatus getLifecycleStatus();

        /*
            Received notification about ROS2-BDI Lifecycle status
        */
        void callbackLifecycleStatus(const ros2_bdi_interfaces::msg::LifecycleStatus::SharedPtr msg)
        {
            if(lifecycle_status_.find(msg->node_name) != lifecycle_status_.end())//key in map, record upd value
                lifecycle_status_[msg->node_name] = msg->status;
        }

        // Selected planning mode
        PlanningMode sel_planning_mode_;
        // agent id that defines the namespace in which the node operates
        std::string agent_id_;
        // step counter
        uint64_t step_counter_;
        // callback to perform main loop of work regularly
        rclcpp::TimerBase::SharedPtr do_work_timer_;
        // work timer interval in ms
        int work_timer_interval_;

        // psys2 nodes active flags
        ros2_bdi_interfaces::msg::PlanningSystemState psys_active_;

        // comm errors psys2
        int psys2_comm_errors_;

        // current known status of the system nodes
        std::map<std::string, uint8_t> lifecycle_status_;
        // Publish updated lifecycle status
        rclcpp::Publisher<ros2_bdi_interfaces::msg::LifecycleStatus>::SharedPtr lifecycle_status_publisher_;
        // Sub to updated lifecycle status
        rclcpp::Subscription<ros2_bdi_interfaces::msg::LifecycleStatus>::SharedPtr lifecycle_status_subscriber_;
        
        // PlanSys2 Monitor Client supporting nodes & clients for calling the {psys2_node}/get_state services
        std::shared_ptr<PlanSysMonitorClient> psys_monitor_client_;
        
        // PlanSys2 state publisher
        rclcpp::Publisher<ros2_bdi_interfaces::msg::PlanningSystemState>::SharedPtr psys_state_publisher_;

}; // PlanSys2Monitor class prototype

#endif // PLANSYS_MONITOR_H_