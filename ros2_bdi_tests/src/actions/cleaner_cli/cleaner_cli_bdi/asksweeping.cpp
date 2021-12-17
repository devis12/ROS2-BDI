#include <string>
#include <vector>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_skills/communications_structs.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

using std::string;
using std::vector;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;

using BDICommunications::UpdOperation;
using BDICommunications::ADD;
using BDICommunications::UpdDesireResult;

class AskSweeping : public BDIActionExecutor
{
    public:
        AskSweeping()
        : BDIActionExecutor("asksweeping", 4)
        {
            sent_ = false;
        }

        float advanceWork()
        {
            float currProgress = getProgress();
            float advancement = 0.0f;
            vector<string> args = getArguments();
            string sweeper_id = args[1];
            string waypoint = args[2];

            if(currProgress == 0.0f)
            {
                sweep_desire_ = buildSweepRequest(waypoint);
                auto reqStatus = sendUpdDesireRequest(sweeper_id, sweep_desire_, ADD, true);
                if(reqStatus.accepted && reqStatus.performed)
                    advancement += 0.015625f;//stop asking, wait for the fulfillment or long enough to know action is failed

                else//desire request not accepted -> fail action    
                    execFailed("Desire to sweep " + waypoint + " has been denied by " + sweeper_id);
                
            
            }else if(isMonitoredDesireSatisfied()){
                advancement = 1.0f - currProgress;//missing part complete
                execSuccess(waypoint + " has been swept given the update coming from " + sweeper_id);
                
            }else if(!isMonitoredDesireSatisfied() && currProgress < 0.96f)
                advancement = 0.015625f;//action still in progress

            else
                execFailed("Waited too much! Sweeper does not seem to do what it's been asked to");    
        
            return advancement;
                
        }

    private:

        // Wrap up the desire to build
        Desire buildSweepRequest(const string& waypoint)
        {
            vector<Belief> target;
            Belief b =  Belief();
            b.pddl_type = Belief().PREDICATE_TYPE;
            b.name = "swept";
            b.params = vector<string>({waypoint});
            target.push_back(b);

            Desire desire = Desire();
            desire.name = "sweep_" + waypoint;
            desire.deadline = 16.4;
            desire.priority = 0.6;
            desire.value = target;

            return desire;
        }

        Desire sweep_desire_;
        bool sent_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<AskSweeping>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
