#ifndef PLANSYS_MONITOR_CLIENT_H_
#define PLANSYS_MONITOR_CLIENT_H_

#include <string>
#include <chrono>
#include <vector>
#include <memory>

#include "lifecycle_msgs/srv/get_state.hpp"
#include "ros2_bdi_core/support/planning_mode.hpp"

#include "rclcpp/rclcpp.hpp"

class PlanSysMonitorClient
{
    public:

        /* 
            Constructor for the supporting nodes for calling the services 
                @nodesBasename is the basename given to the supporting nodes
                @selPlanningMode selected planning mode {OFFLINE, ONLINE}
        */
        PlanSysMonitorClient(const std::string& nodesBasename) : PlanSysMonitorClient(nodesBasename, OFFLINE){};
        /* 
            Constructor for the supporting nodes for calling the services 
                @nodesBasename is the basename given to the supporting nodes
                @selPlanningMode selected planning mode {OFFLINE, ONLINE}
        */
        PlanSysMonitorClient(const std::string& nodesBasename, const PlanningMode& selPlanningMode);
        
        /* Return true if {psysNodeName}/get_state service called confirm that the node is active */
        bool isPsysNodeActive(const std::string& psysNodeName);

        /* Return true if all {psysNodeName}/get_state service called confirm that the nodes are active, wait max_wait in case they're not before returning false */
        bool areAllPsysNodeActive(const std::chrono::seconds max_wait = std::chrono::seconds(0));

    private:

        PlanningMode sel_planning_mode;

        /* Get the reference to the node caller instance for the PlanSys2 node @psys2NodeName */
        rclcpp::Node::SharedPtr getCallerNode(const std::string& psys2NodeName);
        
        /* Get the reference to the client caller instance for the PlanSys2 node @psys2NodeName */
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr getCallerClient(const std::string& psys2NodeName);

        
        // nodes to be spinned while making request (one for each plansys2 node to be called)
        std::vector<rclcpp::Node::SharedPtr> caller_nodes_;

        // below client instances to be instantiated while making a request to ...

        // ... {psys2_node}/get_state
        std::vector<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> caller_clients_;
};

#endif // PLANSYS_MONITOR_CLIENT_H_