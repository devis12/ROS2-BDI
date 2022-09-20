#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class ReqCarrierToCome : public BDIActionExecutor
{
    public:
        ReqCarrierToCome()
        : BDIActionExecutor("req_carrier_to_come", 3,false)
        {}

        float advanceWork()
        {
            return 0.056f;            
        }

    private:
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<ReqCarrierToCome>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}