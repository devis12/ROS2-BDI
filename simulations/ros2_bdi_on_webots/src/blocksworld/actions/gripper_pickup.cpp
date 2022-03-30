#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/msg/string.hpp"

#define ROBOT_NAME_PARAM "robot_name"
#define ROBOT_NAME_PARAM_DEFAULT "gantry"

typedef enum {LOW, CLOSE, HIGH} PickupStatus;

class GripperPickup : public BDIActionExecutor
{
    public:
        GripperPickup()
        : BDIActionExecutor("gripper_pickup", 3, false)
        {
            this->declare_parameter(ROBOT_NAME_PARAM, ROBOT_NAME_PARAM_DEFAULT);
            robot_name_ = this->get_parameter(ROBOT_NAME_PARAM).as_string();
            gripper_pose_cmd_publisher_ = this->create_publisher<example_interfaces::msg::String>("/"+robot_name_+"/cmd_gripper_pose", rclcpp::QoS(1).keep_all());
            gripper_status_cmd_publisher_ = this->create_publisher<example_interfaces::msg::String>("/"+robot_name_+"/cmd_gripper_status", rclcpp::QoS(1).keep_all());
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            action_status_ = LOW;
            repeat_ = 0;
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
            msg.data = (action_status_ == LOW)? "low"   : 
                        ( 
                            (action_status_ == CLOSE)? "close" :
                            "high"
                        );
            
            if(action_status_ == CLOSE)
            {   
                gripper_status_cmd_publisher_->publish(msg);
            }
            else
            {
                gripper_pose_cmd_publisher_->publish(msg);
            }
            
            repeat_++;
            if(repeat_ == 3)
            {
                //publish same cmd for three action steps then switch to new status
                repeat_ = 0;
                action_status_ = (action_status_ == LOW)? CLOSE : HIGH;
            }

            return 0.112f;            
        }

    private:
        PickupStatus action_status_;
        uint8_t repeat_;
        rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr gripper_pose_cmd_publisher_;
        rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr gripper_status_cmd_publisher_;
        std::string robot_name_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<GripperPickup>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
