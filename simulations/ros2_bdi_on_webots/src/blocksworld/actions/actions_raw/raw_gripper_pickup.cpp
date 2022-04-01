#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

// #include "example_interfaces/msg/string.hpp"

#define ROBOT_NAME_PARAM "robot_name"
#define ROBOT_NAME_PARAM_DEFAULT "gantry"

typedef enum {LOW, CLOSE, HIGH} PickupStatus;

class GripperPickupRaw : public plansys2::ActionExecutorClient
{
    public:
        GripperPickupRaw()
        : plansys2::ActionExecutorClient("gripper_pickup", std::chrono::milliseconds(333))
        {
            //this->declare_parameter(ROBOT_NAME_PARAM, ROBOT_NAME_PARAM_DEFAULT);
            this->set_parameter(rclcpp::Parameter("action_name", "gripper_pickup"));
            /*
            robot_name_ = this->get_parameter(ROBOT_NAME_PARAM).as_string();
            gripper_pose_cmd_publisher_ = this->create_publisher<example_interfaces::msg::String>("/"+robot_name_+"/cmd_gripper_pose", rclcpp::QoS(1).keep_all());
            gripper_status_cmd_publisher_ = this->create_publisher<example_interfaces::msg::String>("/"+robot_name_+"/cmd_gripper_status", rclcpp::QoS(1).keep_all());
            */
           progress_ = 0.0f;
            this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        }
        /*
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            action_status_ = LOW;
            repeat_ = 0;
            progress_ = 0;
            gripper_pose_cmd_publisher_->on_activate();
            gripper_status_cmd_publisher_->on_activate();

            return plansys2::ActionExecutorClient::on_activate(previous_state);
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {
            gripper_pose_cmd_publisher_->on_deactivate();
            gripper_status_cmd_publisher_->on_deactivate();

            return plansys2::ActionExecutorClient::on_deactivate(previous_state);
        }
        */
        void do_work()
        {
            std::string action_full_name = get_action_name() + " " + get_arguments()[1] + " " + get_arguments()[2];

            if(progress_ > 1.0)
                progress_ = 0.0f;
            
            progress_ += 0.1;
            std::cout << action_full_name << " curr progress " << progress_ << std::endl;
            send_feedback(progress_, action_full_name + ": "+ std::to_string(progress_));
            if(progress_ >= 1.0)
                finish(true, 1.0, action_full_name + ": successful execution");
            /*
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

            std::string action_full_name = get_action_name() + " " + get_arguments()[1] + " " + get_arguments()[2] + " " + get_arguments()[3];


            progress_ += 0.112f;
            if(progress_ < 1)
                send_feedback(progress_, action_full_name + ": " + std::to_string(progress_));
            else
                finish(true, 1.0, action_full_name + ": successful execution");
            */
        }

    private:
        float progress_;
        PickupStatus action_status_;
        uint8_t repeat_;
        // rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr gripper_pose_cmd_publisher_;
        // rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr gripper_status_cmd_publisher_;
        std::string robot_name_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<GripperPickupRaw>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
