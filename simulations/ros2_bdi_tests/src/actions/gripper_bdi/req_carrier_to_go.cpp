#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class ReqCarrierToGo : public BDIActionExecutor
{
    public:
        ReqCarrierToGo()
        : BDIActionExecutor("req_carrier_to_go", 3, false)
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
  
  auto actionNode = std::make_shared<ReqCarrierToGo>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}