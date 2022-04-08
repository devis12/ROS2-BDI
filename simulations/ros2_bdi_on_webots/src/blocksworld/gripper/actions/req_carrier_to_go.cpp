#include "rclcpp/rclcpp.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;

class ReqCarrierToGo : public BDIActionExecutor
{
    public:
        ReqCarrierToGo()
        : BDIActionExecutor("req_carrier_to_go", 1, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();
        }

        float advanceWork()
        {
            float step_progress = 0.0;
            float action_progress = getProgress();
            auto carrier_id = getArguments()[0];
            
            if(action_progress < 0.15)
            {
                auto deposit_id = getArguments()[2];
                deposit_id = deposit_id.substr(0, deposit_id.size()-2);//remove last chars id (does not need it -> every carrier knows just base/deposit it's attached to)
                
                requested_desire_ = buildDesire(carrier_id, deposit_id);
                BDICommunications::UpdDesireResult result = sendUpdDesireRequest(carrier_id, requested_desire_, BDICommunications::ADD, true);
                if (result.desire.name == requested_desire_.name)
                    step_progress += (result.accepted && result.performed)? 0.15 : 0.0; //advance and wait for its completion 
            }
            else if (isMonitoredDesireFulfilled(carrier_id, requested_desire_))
                execSuccess();//
            else if (action_progress >= 0.899)
                execFailed("Too much time waited for the carrier to fulfill the desire");//
            else
            {
                //desire request already made
                step_progress += 0.1;
            }

            return step_progress;
        }

    private:
        Desire buildDesire(const std::string& agent_id, const std::string& target)
        {
            auto desire = Desire();
            desire.name = robot_name_ + "_goto_dep";
            desire.deadline = 6.0;//TODO develop API to retrieve duration from domain definition
            desire.priority = 0.6;
            auto value = Belief();
            {
                //Building value belief
                value.name = "in";
                value.pddl_type = value.PREDICATE_TYPE;
                value.params = {agent_id, target};
            }
            desire.value = {value};//desire's value is a list of beliefs, even if here we need just one
            return desire;
        }

        std::string robot_name_;
        Desire requested_desire_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<ReqCarrierToGo>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
