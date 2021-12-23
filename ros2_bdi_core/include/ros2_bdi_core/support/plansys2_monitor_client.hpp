#ifndef PLANSYS2_MONITOR_CLIENT_H_
#define PLANSYS2_MONITOR_CLIENT_H_

#include <string>
#include <vector>
#include <memory>

#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

class PlanSys2MonitorClient
{
    public:
        /* 
            Constructor for the supporting nodes for calling the services 
                @nodesBasename is the basename given to the supporting nodes
        */
        PlanSys2MonitorClient(const std::string& nodesBasename);
        
        /* Return true if {psys2NodeName}/get_state service called confirm that the node is active */
        bool isPsys2NodeActive(const std::string& psys2NodeName);

    private:
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

#endif // PLANSYS2_MONITOR_CLIENT_H_