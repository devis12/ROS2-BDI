#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/msg/string.hpp"

class GripperPickup : public BDIActionExecutor
{
    public:
        GripperPickup()
        : BDIActionExecutor("gripper_pickup", 2, false)
        {
            gripper_pose_cmd_publisher_ = this->create_publisher<example_interfaces::msg::String>("/cmd_gripper_pose", rclcpp::QoS(1).keep_all());
            gripper_status_cmd_publisher_ = this->create_publisher<example_interfaces::msg::String>("/cmd_gripper_status", rclcpp::QoS(1).keep_all());
            action_status_ = 0;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            
            gripper_pose_cmd_publisher_->on_activate();
            gripper_status_cmd_publisher_->on_activate();

            return BDIActionExecutor::on_activate(previous_state);
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {
            gripper_pose_cmd_publisher_->on_deactivate();
            gripper_status_cmd_publisher_->on_deactivate();

            return BDIActionExecutor::on_deactivate(previous_state);
        }

        float advanceWork()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = (action_status_ < 4)? "low"   : 
                        ( 
                            (action_status_ < 8)? "close" :
                            "high"
                        );
            
            if(msg.data == "close")
            {   
                gripper_status_cmd_publisher_->publish(msg);
            }
            else
            {
                gripper_pose_cmd_publisher_->publish(msg);
            }
            
            action_status_++;

            return 0.08f;            
        }

    private:
        uint8_t action_status_;
        rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr gripper_pose_cmd_publisher_;
        rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr gripper_status_cmd_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<GripperPickup>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
