#include "ros2_bdi_on_litter_world/params.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "litter_world_interfaces/action/cmd_load.hpp"
#include "litter_world_interfaces/action/cmd_pose.hpp"


using std::string;
using std::placeholders::_1;
using std::placeholders::_2;

using litter_world_interfaces::msg::Pose;
using litter_world_interfaces::action::CmdLoad;
using litter_world_interfaces::action::CmdPose;
using GoalHandleCmdPose = rclcpp_action::ClientGoalHandle<CmdPose>;
using GoalHandleCmdLoad = rclcpp_action::ClientGoalHandle<CmdLoad>;

class LitterPickup : public BDIActionExecutor
{
    public:
        LitterPickup()
        : BDIActionExecutor("litter_pickup", 2)
        {
            string robot_name = this->get_parameter("agent_id").as_string();
            litter_remove_action_name_ = "/cmd_remove_litter";
            this->client_cmd_pose_ptr_ = rclcpp_action::create_client<CmdPose>(
                this->get_node_base_interface(),
                this->get_node_graph_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                litter_remove_action_name_);
            
            litter_load_action_name_ = "/cmd_" + robot_name + "_hold";
            this->client_cmd_load_ptr_ = rclcpp_action::create_client<CmdLoad>(
                this->get_node_base_interface(),
                this->get_node_graph_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                litter_load_action_name_);
            
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            litter_removed_goal_sent_ = false;
            litter_removed_goal_accepted_ = false;
            litter_removed_ = false;
            litter_load_goal_sent_ = false;
            litter_load_goal_accepted_ = false;

            return BDIActionExecutor::on_activate(previous_state);
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {
            return BDIActionExecutor::on_deactivate(previous_state);
        }

        float advanceWork()
        {
            float step_progress = 0.0f;
            
            // STEP1: removing litter
            if(!litter_removed_goal_sent_ && step_progress == 0.0f)
            {
                string litter = getArguments()[1];
                string litterPosition = getArguments()[2];
                sendLitterRemoveGoal(extractPose(litterPosition));
            }
            else if(litter_removed_goal_sent_ && getProgress() < 0.1)
            {
                step_progress = 0.1f;
            }
            else if(litter_removed_goal_accepted_ && getProgress() < 0.2)
            {
                step_progress = 0.1f;
            }

            // STEP2: loading litter
            if(litter_removed_)
            {
                if(!litter_load_goal_sent_ && step_progress == 0.0f)
                {
                    string litter = getArguments()[1];
                    string litterPosition = getArguments()[2];
                    sendLitterLoadGoal(litter, extractPose(litterPosition));
                }
                else if(litter_load_goal_sent_ && getProgress() < 0.1)
                {
                    step_progress = 0.1f;
                }
                else if(litter_load_goal_accepted_ && getProgress() < 0.2)
                {
                    step_progress = 0.1f;
                } 
            }

            return step_progress;            
        }

    private:

        /*
            From cell name in format "c_x_y", extract x and y, embedding them into a Pose obj
        */
        Pose extractPose (string cell)
        {
            Pose res = Pose{};
            std::size_t firstU = cell.find_first_of("_");
            std::size_t secondU = cell.find_last_of("_");
            if(firstU != string::npos && secondU != string::npos)
            {
                if(firstU + 1 < cell.length() && secondU + 1 < cell.length())
                {
                    res.x = std::stoi(cell.substr(firstU+1, secondU - firstU - 1));
                    res.y = std::stoi(cell.substr(secondU + 1));
                }
            }

            return res;
        }

        void sendLitterRemoveGoal(Pose litterPosition)
        {
            if (!this->client_cmd_pose_ptr_->wait_for_action_server(std::chrono::seconds(LWACTIONS_WAIT_TIMEOUT))) {
                RCLCPP_ERROR(this->get_logger(), "Action server " + litter_remove_action_name_ + " not available after waiting");
                rclcpp::shutdown();
                return;
            }

            auto goal_msg = CmdPose::Goal();
            goal_msg.cmd.x = litterPosition.x;
            goal_msg.cmd.y = litterPosition.y;

            auto send_goal_options = rclcpp_action::Client<CmdPose>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&LitterPickup::litter_removed_goal_response_callback, this, _1);
            send_goal_options.feedback_callback = std::bind(&LitterPickup::litter_removed_feedback_callback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&LitterPickup::litter_removed_result_callback, this, _1);
            this->client_cmd_pose_ptr_->async_send_goal(goal_msg, send_goal_options);
            litter_removed_goal_sent_ = true;
        }

        void litter_removed_goal_response_callback(std::shared_future<GoalHandleCmdPose::SharedPtr> future)
        {
            auto goal_handle = future.get();
            if (!goal_handle) 
                execFailed("Litter removed goal was rejected!");
            else
                litter_removed_goal_accepted_ = true;
        }

        void litter_removed_feedback_callback(
            GoalHandleCmdPose::SharedPtr,
            const std::shared_ptr<const CmdPose::Feedback> feedback_msg)
        {
            //TODO progress
        }

        void litter_removed_result_callback(const GoalHandleCmdPose::WrappedResult & result)
        {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    execFailed(); //RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    execFailed(); //RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    return;
                default:
                    execFailed(); //RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
            }
            
            if(result.result->performed)
                litter_removed_ = true;
            else
                execFailed();
        }

        void sendLitterLoadGoal(string litter, Pose litterPosition)
        {
            if (!this->client_cmd_load_ptr_->wait_for_action_server(std::chrono::seconds(LWACTIONS_WAIT_TIMEOUT))) {
                RCLCPP_ERROR(this->get_logger(), "Action server " + litter_load_action_name_ + " not available after waiting");
                rclcpp::shutdown();
                return;
            }

            auto goal_msg = CmdLoad::Goal();
            goal_msg.cmd = "load";

            auto send_goal_options = rclcpp_action::Client<CmdLoad>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&LitterPickup::litter_load_goal_response_callback, this, _1);
            send_goal_options.feedback_callback = std::bind(&LitterPickup::litter_load_feedback_callback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&LitterPickup::litter_load_result_callback, this, _1);
            this->client_cmd_load_ptr_->async_send_goal(goal_msg, send_goal_options);
            litter_load_goal_sent_ = true;
        }

        void litter_load_goal_response_callback(std::shared_future<GoalHandleCmdLoad::SharedPtr> future)
        {
            auto goal_handle = future.get();
            if (!goal_handle) 
                execFailed("Litter load goal was rejected!");
            else
                litter_load_goal_accepted_ = true;
        }

        void litter_load_feedback_callback(
            GoalHandleCmdLoad::SharedPtr,
            const std::shared_ptr<const CmdLoad::Feedback> feedback_msg)
        {
            //TODO progress
        }

        void litter_load_result_callback(const GoalHandleCmdLoad::WrappedResult & result)
        {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    execFailed(); //RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    execFailed(); //RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    return;
                default:
                    execFailed(); //RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
            }
            
            if(result.result->performed)
                execSuccess();
            else
                execFailed();
        }

        bool litter_removed_goal_sent_;
        bool litter_removed_goal_accepted_;
        bool litter_removed_;
        bool litter_load_goal_sent_;
        bool litter_load_goal_accepted_;
        string litter_remove_action_name_;
        string litter_load_action_name_;
        rclcpp_action::Client<CmdPose>::SharedPtr client_cmd_pose_ptr_;
        rclcpp_action::Client<CmdLoad>::SharedPtr client_cmd_load_ptr_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<LitterPickup>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}