#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/msg/string.hpp"

class MoveGripper : public BDIActionExecutor
{
    public:
        MoveGripper()
        : BDIActionExecutor("move_gripper", 2, false)
        {
            move_gripper_cmd_publisher_ = this->create_publisher<example_interfaces::msg::String>("/cmd_motors_pose", rclcpp::QoS(1).keep_all());
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            move_gripper_cmd_publisher_->on_activate();

            return BDIActionExecutor::on_activate(previous_state);
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {
            move_gripper_cmd_publisher_->on_deactivate();

            return BDIActionExecutor::on_deactivate(previous_state);
        }

        float advanceWork()
        {
            std::string destination = getArguments()[2];
            auto msg = example_interfaces::msg::String();
            msg.data = destination;
            move_gripper_cmd_publisher_->publish(msg);
            return 0.08f;            
        }

    private:
        rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr move_gripper_cmd_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<MoveGripper>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
