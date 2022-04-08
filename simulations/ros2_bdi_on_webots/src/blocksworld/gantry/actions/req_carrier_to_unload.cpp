#include "rclcpp/rclcpp.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"

#include "example_interfaces/msg/string.hpp"

class CarrierUnload : public BDIActionExecutor
{
    public:
        CarrierUnload()
        : BDIActionExecutor("req_carrier_to_unload", 2, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            return BDIActionExecutor::on_activate(previous_state);
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {

            return BDIActionExecutor::on_deactivate(previous_state);
        }

        float advanceWork()
        {

            return 0.5f;            
        }

    private:
        std::string robot_name_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<CarrierUnload>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
