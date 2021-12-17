#ifndef PLANSYS2_MONITOR_H_
#define PLANSYS2_MONITOR_H_

#include <string>
#include <memory>

#include "ros2_bdi_interfaces/msg/plan_sys2_state.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"
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
            Build PlanSys2 state msg wrt the current state of the respective boolean flags
        */
        ros2_bdi_interfaces::msg::PlanSys2State buildPlanSys2StateMsg();

        /*
            Activate thread to check active state of plansys2 node (planner, domain_expert, problem_expert)
        */
        void checkPsys2NodeActive(const std::string& psys2NodeName);

        /*
            Check with a srv call to its respective get_state service if the plansys2 planner/domain_expert/problem_expert is active
            @psys2NodeName is equal to one of the plansys2 nodes you want to check if active through get_state srv
        */
        void checkPsys2NodeActiveThread(const std::string& psys2NodeName);

        
        // agent id that defines the namespace in which the node operates
        std::string agent_id_;
        // callback to perform main loop of work regularly
        rclcpp::TimerBase::SharedPtr do_work_timer_;
        // work timer interval in ms
        int work_timer_interval_;

        // flag to denote if the problem expert node seems to be up and active
        bool psys2_problem_expert_active_;
        // flag to denote if the domain expert node seems to be up and active
        bool psys2_domain_expert_active_;
        // flag to denote if the planner node seems to be up and active
        bool psys2_planner_active_;
        // flag to denote if the executor node seems to be up and active
        bool psys2_executor_active_;

        // comm errors psys2
        int psys2_comm_errors_;

        std::shared_ptr<std::thread> chk_problem_expert_thread_;
        std::shared_ptr<std::thread> chk_domain_expert_thread_;
        std::shared_ptr<std::thread> chk_planner_thread_;
        std::shared_ptr<std::thread> chk_executor_thread_;

        rclcpp::Publisher<ros2_bdi_interfaces::msg::PlanSys2State>::SharedPtr psys2_state_publisher_;//PlanSys2 state publisher

}; // PlanSys2Monitor class prototype

#endif // PLANSYS2_MONITOR_H_