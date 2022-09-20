#include "ros2_bdi_on_litter_world/params.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "litter_world_interfaces/action/cmd_pose.hpp"


using std::string;
using std::placeholders::_1;
using std::placeholders::_2;

using litter_world_interfaces::msg::Pose;
using litter_world_interfaces::action::CmdPose;
using GoalHandleCmdPose = rclcpp_action::ClientGoalHandle<CmdPose>;

#define MAX_FAILURES 0

class Move : public BDIActionExecutor
{
    public:
        Move()
        : BDIActionExecutor("move", 2)
        {
            string robot_name = this->get_parameter("agent_id").as_string();
            action_name_ = "/cmd_" + robot_name + "_move";
            
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            failures_ = 0;
            goal_sent_ = false;
            goal_accepted_ = false;
            this->client_cmd_pose_ptr_ = rclcpp_action::create_client<CmdPose>(
                this->get_node_base_interface(),
                this->get_node_graph_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                action_name_);
            return BDIActionExecutor::on_activate(previous_state);
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {
            this->client_cmd_pose_ptr_.reset();
            return BDIActionExecutor::on_deactivate(previous_state);
        }

        float advanceWork()
        {
            float step_progress = 0.0f;
            
            if(!goal_sent_)
            {
                string currentPosition = getArguments()[1];
                string goalPosition = getArguments()[2];
                sendGoal(extractPose(currentPosition), extractPose(goalPosition));
            }
            else if(goal_sent_ && getProgress() < 0.9)
            {
                step_progress = 0.05f;
            }
            else if(goal_accepted_ && getProgress() < 0.9)
            {
                step_progress = 0.05f;
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

        void sendGoal(Pose currentCell, Pose goalCell)
        {
            if (!this->client_cmd_pose_ptr_->wait_for_action_server(std::chrono::seconds(LWACTIONS_WAIT_TIMEOUT))) {
                RCLCPP_ERROR(this->get_logger(), "Action server " + action_name_ + " not available after waiting");
                rclcpp::shutdown();
                return;
            }

            auto goal_msg = CmdPose::Goal();
            goal_msg.cmd.x = goalCell.x - currentCell.x;
            goal_msg.cmd.y = goalCell.y - currentCell.y;

            auto send_goal_options = rclcpp_action::Client<CmdPose>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&Move::goal_response_callback, this, _1);
            send_goal_options.feedback_callback = std::bind(&Move::feedback_callback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&Move::result_callback, this, _1);
            this->client_cmd_pose_ptr_->async_send_goal(goal_msg, send_goal_options);
            goal_sent_ = true;
        }

        void goal_response_callback(std::shared_future<GoalHandleCmdPose::SharedPtr> future)
        {
            auto goal_handle = future.get();
            if (!goal_handle) 
            {
                goal_sent_ = false;
                failures_++;
                if(failures_ > MAX_FAILURES) execFailed("Goal was rejected!");
            }
            else
                goal_accepted_ = true;
        }

        void feedback_callback(
            GoalHandleCmdPose::SharedPtr,
            const std::shared_ptr<const CmdPose::Feedback> feedback_msg)
        {
            //TODO progress
        }

        void result_callback(const GoalHandleCmdPose::WrappedResult & result)
        {
            bool failed = false;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    failed = true; //RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    failed = true; //RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    return;
                default:
                    failed = true; //RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
            }
            
            if(result.result->performed)
                execSuccess();
            else
                failed = true;
            
            if(failed)
            {
                failures_++;
                if(failures_ > MAX_FAILURES)
                    execFailed("Too many failures encountered");
                else
                {
                    goal_sent_ = false;
                    goal_accepted_ = false;
                }
            }
        }

        int failures_;
        bool goal_sent_;
        bool goal_accepted_;
        string action_name_;
        rclcpp_action::Client<CmdPose>::SharedPtr client_cmd_pose_ptr_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<Move>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
