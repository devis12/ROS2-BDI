#ifndef RECYCLE_H_
#define RECYCLE_H_

#include "rclcpp/rclcpp.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "litter_world_interfaces/action/cmd_load.hpp"

class Recycle : public BDIActionExecutor
{
    public:
        Recycle(const std::string& litter_type);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            goal_sent_ = false;
            goal_accepted_ = false;
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
            
            if(!goal_sent_ && step_progress == 0.0f)
            {
                sendGoal();
            }
            else if(goal_sent_)
            {
                step_progress = 0.1f;
            }
            else if(goal_accepted_)
            {
                step_progress = 0.1f;
            }

            return step_progress;            
        }
    private:

        void sendGoal();

        void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<litter_world_interfaces::action::CmdLoad>::SharedPtr> future);

        void feedback_callback(
            rclcpp_action::ClientGoalHandle<litter_world_interfaces::action::CmdLoad>::SharedPtr,
            const std::shared_ptr<const litter_world_interfaces::action::CmdLoad::Feedback> feedback_msg);

        void result_callback(const rclcpp_action::ClientGoalHandle<litter_world_interfaces::action::CmdLoad>::WrappedResult & result);

        bool goal_sent_;
        bool goal_accepted_;
        std::string litter_unload_action_name_;
        std::string litter_type_;
        rclcpp_action::Client<litter_world_interfaces::action::CmdLoad>::SharedPtr client_cmd_unload_ptr_;

};

#endif // RECYCLE_H_
