#ifndef JAVAFF_CLIENT_H_
#define JAVAFF_CLIENT_H_

#include <string>
#include <memory>

#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "javaff_interfaces/srv/java_ff_plan.hpp"

#include "rclcpp/rclcpp.hpp"

class JavaFFClient
{
    public:
        /* Constructor for the supporting node for calling javaff services */
        JavaFFClient(const std::string& nodeBasename);

        bool launchPlanSearch(const std::string& problem, const int& interval);

    private:
        bool makePlanSearchRequest(const javaff_interfaces::srv::JavaFFPlan::Request::SharedPtr& request);


        // node to be spinned while making request to javaff srvs 
        rclcpp::Node::SharedPtr caller_node_;

        // client instance to make the request to the javaff_server/start_plan srv
        rclcpp::Client<javaff_interfaces::srv::JavaFFPlan>::SharedPtr start_plan_client_;

};

#endif //JAVAFF_CLIENT_H_