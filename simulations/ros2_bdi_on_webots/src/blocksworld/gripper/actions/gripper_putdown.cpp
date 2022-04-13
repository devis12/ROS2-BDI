#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"

#include "example_interfaces/msg/string.hpp"
#include "webots_ros2_simulations_interfaces/msg/move_status.hpp"

using std::string;

using example_interfaces::msg::String;
using webots_ros2_simulations_interfaces::msg::MoveStatus;

typedef enum {LOW, OPEN, HIGH} PutdownStatus;

class GripperPutdown : public BDIActionExecutor
{
    public:
        GripperPutdown()
        : BDIActionExecutor("gripper_putdown", 3)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();
            gripper_pose_cmd_publisher_ = this->create_publisher<String>("/"+robot_name_+"/cmd_gripper_pose", rclcpp::QoS(1).keep_all());
            gripper_status_cmd_publisher_ = this->create_publisher<String>("/"+robot_name_+"/cmd_gripper_status", rclcpp::QoS(1).keep_all());
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
            auto msg = String();
            msg.data = (action_status_ == LOW)? "low"   : 
                        ( 
                            (action_status_ == OPEN)? "open" :
                            "high"
                        );
            
            if(action_status_ == OPEN)
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
                action_status_ = (action_status_ == LOW)? OPEN : HIGH;
            }

            return 0.112f;            
        }

    private:
        PutdownStatus action_status_;
        uint8_t repeat_;
        rclcpp_lifecycle::LifecyclePublisher<String>::SharedPtr gripper_pose_cmd_publisher_;
        rclcpp_lifecycle::LifecyclePublisher<String>::SharedPtr gripper_status_cmd_publisher_;
        rclcpp::Subscription<MoveStatus>::SharedPtr gripper_move_status_subscriber_;
        string robot_name_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<GripperPutdown>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
