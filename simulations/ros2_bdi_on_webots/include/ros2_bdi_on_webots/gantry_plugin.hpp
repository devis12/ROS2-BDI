#ifndef WEBOTS_ROS2_GANTRY_PLUGIN_HPP
#define WEBOTS_ROS2_GANTRY_PLUGIN_HPP

#include <vector>

#include "rclcpp/macros.hpp"
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"


namespace webots_ros2_my_plugins
{
  class WebotsRos2GantryPlugin : public webots_ros2_driver::PluginInterface
  {
    public:
        // Your plugin has to override step() and init() methods
        void step() override;
        void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    
    private:
        webots::Supervisor* robot_;
        std::vector<webots::Motor*> wheel_motors_;
        std::vector<webots::Motor*> grip_motors_;
        webots::Motor* bridge_motor_;
        webots::Motor* turret_motor_;
        webots::Motor* lift_motor_;

        rclcpp::Node::SharedPtr interface_node_;
  };
}
#endif