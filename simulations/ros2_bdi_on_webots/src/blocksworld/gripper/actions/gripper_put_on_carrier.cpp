#include <utility>
#include <map>

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"

#include "example_interfaces/msg/string.hpp"
#include "webots_ros2_simulations_interfaces/msg/move_status.hpp"

#define BASE_STEP_PROGRESS 0.0625f
#define MEANINGFUL_DIFF 0.005
#define GRIPPER_HIGH_MIN_H 1.4 // min h in the z value of the current pose to consider the gripper in high position


using std::string;
using std::map;

using ros2_bdi_interfaces::msg::Belief;
using BDICommunications::UpdBeliefResult;

using example_interfaces::msg::String;
using webots_ros2_simulations_interfaces::msg::MoveStatus;

typedef enum {FIX_POSE, LOW, OPEN, HIGH} PutdownStatus;

const string bases[3] = {"base_a", "base_b", "base_c"};

class GripperPutOnCarrier : public BDIActionExecutor
{
    public:
        GripperPutOnCarrier()
        : BDIActionExecutor("gripper_put_on_carrier", 2)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            gripper_pose_cmd_publisher_ = this->create_publisher<String>("/"+robot_name_+"/cmd_gripper_pose", rclcpp::QoS(1).reliable());
            gripper_status_cmd_publisher_ = this->create_publisher<String>("/"+robot_name_+"/cmd_gripper_status", rclcpp::QoS(1).reliable());
            move_gripper_cmd_publisher_ = this->create_publisher<String>("/"+robot_name_+"/cmd_motors_pose", 
                rclcpp::QoS(1).reliable());

            gripper_move_status_subscriber_ = this->create_subscription<MoveStatus>("/"+robot_name_+"/motors_move_status", 
                rclcpp::QoS(5).best_effort(),
                std::bind(&GripperPutOnCarrier::gripperMoveStatusCallback, this, std::placeholders::_1));

            for(auto b : bases)//init to 0 the counters for all the bases
                base_counter_[b] = 0;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            action_status_ = FIX_POSE;
            repeat_ = 0;
            prev_z_ = 0.0f;
            curr_z_ = 0.0f;
            last_box_loaded_feedback_.accepted = false;
            last_box_loaded_feedback_.performed = false;
            gripper_pose_cmd_publisher_->on_activate();
            gripper_status_cmd_publisher_->on_activate();
            move_gripper_cmd_publisher_->on_activate();

            return BDIActionExecutor::on_activate(previous_state);
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {
            gripper_pose_cmd_publisher_->on_deactivate();
            gripper_status_cmd_publisher_->on_deactivate();
            move_gripper_cmd_publisher_->on_deactivate();

            return BDIActionExecutor::on_deactivate(previous_state);
        }

        float advanceWork()
        {
            float step_progress = BASE_STEP_PROGRESS;//action should complete in 16 steps (4 for fixing pose, 4 for low, 4 for open, 4 for high and notify carrier)
            auto msg = String();

            if(repeat_ < 4)
            {
                //fix pose
                auto base = getArguments()[3];
                if(base_counter_.find(base) != base_counter_.end())
                {
                    if(repeat_ == 0)
                        base_counter_[base]++;//increment just the first repeat
                    
                    base += (base_counter_[base]%2 == 0)? "2" : "1";
                }
                msg.data = base;
                move_gripper_cmd_publisher_->publish(msg);
            }
            else
            {
                //put down, release and go back up
                if(action_status_ == FIX_POSE)
                    action_status_ = LOW;

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

                if((repeat_+1)%4 == 0 && std::abs(curr_z_ - prev_z_) < MEANINGFUL_DIFF)//publish same cmd for three action steps then switch to new status if z (height pos.) is stable
                    action_status_ = (action_status_ == LOW)? OPEN : HIGH;
            }

            // inform carrier about the status of the put on carrier operation
            auto carrier_id = getArguments()[2];
            auto last_box_loaded = Belief();
            last_box_loaded.name = "lb_load_completed";
            last_box_loaded.pddl_type = last_box_loaded.PREDICATE_TYPE;
            last_box_loaded.params = {carrier_id};
            if(repeat_ < 15)
            {
                //notify the carrier loading box has not been completed yet
                sendUpdBeliefRequest(carrier_id, last_box_loaded, BDICommunications::DEL);
            }
            else
            {
                 if(action_status_ == HIGH && getProgress() >= BASE_STEP_PROGRESS*15)
                 {
                    if(curr_z_ > (GRIPPER_HIGH_MIN_H) && std::abs(curr_z_ - prev_z_) < MEANINGFUL_DIFF && //height of the gripper should ne high enough and stable (high operation after put down has been completed)
                        (!last_box_loaded_feedback_.accepted || !last_box_loaded_feedback_.performed))//notify the carrier it is ready to go at the very last step)
                    {
                        last_box_loaded_feedback_ = sendUpdBeliefRequest(carrier_id, last_box_loaded, BDICommunications::ADD);
                        step_progress = (last_box_loaded_feedback_.accepted && last_box_loaded_feedback_.performed)? BASE_STEP_PROGRESS : 0.0f;//wait for the carrier to accept the message of loading box completed
                    }
                    else
                    {
                        step_progress = 0.0f;//wait for the gripper to come up completely before notifying the carrier the loading has been completed
                    }

                 } 
                    
            }
            

            repeat_++;

            return step_progress;            
        }

    private:

        /*gripper move status to update z*/
        void gripperMoveStatusCallback(const MoveStatus::SharedPtr msg)
        {
            prev_z_ = curr_z_;
            curr_z_ = msg->current_pos.z;
        }

        PutdownStatus action_status_;
        uint8_t repeat_;
        rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr gripper_pose_cmd_publisher_;
        rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr gripper_status_cmd_publisher_;
        rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr move_gripper_cmd_publisher_;
        rclcpp::Subscription<webots_ros2_simulations_interfaces::msg::MoveStatus>::SharedPtr gripper_move_status_subscriber_;
        double prev_z_, curr_z_;//prev & current z value of the gripper to know if the low/high action is completed
        map<string, int> base_counter_;//how many boxes haved you put down in a given base
        UpdBeliefResult last_box_loaded_feedback_;
        string robot_name_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<GripperPutOnCarrier>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
