#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cmath"

// #include "example_interfaces/msg/string.hpp"
// #include "webots_ros2_simulations_interfaces/msg/move_status.hpp"
// #include "geometry_msgs/msg/point.hpp"

#define MEANINGFUL_DIFF 0.001
#define ROBOT_NAME_PARAM "robot_name"
#define ROBOT_NAME_PARAM_DEFAULT "gantry"

class GripperMoveRaw : public plansys2::ActionExecutorClient
{
    public:
        GripperMoveRaw()
        : plansys2::ActionExecutorClient("gripper_move", std::chrono::milliseconds(500))
        {
            //this->declare_parameter(ROBOT_NAME_PARAM, ROBOT_NAME_PARAM_DEFAULT);
            this->set_parameter(rclcpp::Parameter("action_name", "gripper_move"));
            /*
            robot_name_ = this->get_parameter(ROBOT_NAME_PARAM).as_string();
            move_gripper_cmd_publisher_ = this->create_publisher<example_interfaces::msg::String>("/"+robot_name_+"/cmd_motors_pose", 
                rclcpp::QoS(1).keep_all());
            */      
           progress_ = 0.0f;
            this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        }

        /*
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            move_gripper_cmd_publisher_->on_activate();
            
            gantry_move_status_subscriber_ = this->create_subscription<webots_ros2_simulations_interfaces::msg::MoveStatus>("/"+robot_name_+"/motors_move_status", 
                rclcpp::QoS(5).best_effort(),
                std::bind(&GripperMoveRaw::gantryMoveStatusCallback, this, std::placeholders::_1));

            last_step_progress_info_ = 0.0f;

            return plansys2::ActionExecutorClient::on_activate(previous_state);
        }*/
        /*
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {
            move_gripper_cmd_publisher_->on_deactivate();

            return plansys2::ActionExecutorClient::on_deactivate(previous_state);
        }*/

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

            /*float step_progress = 0.0f;
            std::string destination = get_arguments()[2];
            
            if(last_step_progress_info_ > 0.0 && destination != move_status_.target_name)//action execution had started, but it somehow was interrupted by some other controllers
                finish(false, last_step_progress_info_, "gripper_move failed execution");

            else if (last_step_progress_info_ == 0.0 && destination != move_status_.target_name)//move cmd to trigger action execution hasn't been given yet 
            {
                auto msg = example_interfaces::msg::String();
                msg.data = destination;
                move_gripper_cmd_publisher_->publish(msg);
            }
            else if (destination == move_status_.target_name) 
            {
                step_progress = move_status_.progress - last_step_progress_info_;
                last_step_progress_info_ = move_status_.progress;
                
                std::string action_full_name = get_action_name() + " " + get_arguments()[1] + " " + get_arguments()[2];

                if(move_status_.progress == 1.0 || move_status_.progress > 0.975 && step_progress < MEANINGFUL_DIFF)
                    finish(true, 1.0, action_full_name + ": successful execution");
                else
                    send_feedback(move_status_.progress, action_full_name + ": "+ std::to_string(move_status_.progress));
            }*/


        }

    private:

        // void gantryMoveStatusCallback(const webots_ros2_simulations_interfaces::msg::MoveStatus::SharedPtr msg)
        // {
        //     move_status_ = *msg;
        // }

        // rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr move_gripper_cmd_publisher_;
        // rclcpp::Subscription<webots_ros2_simulations_interfaces::msg::MoveStatus>::SharedPtr gantry_move_status_subscriber_;
        float last_step_progress_info_;
        //webots_ros2_simulations_interfaces::msg::MoveStatus move_status_;
        std::string robot_name_;
        float progress_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<GripperMoveRaw>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
