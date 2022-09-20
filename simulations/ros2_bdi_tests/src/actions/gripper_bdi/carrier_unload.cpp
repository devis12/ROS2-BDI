#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class CarrierUnload : public BDIActionExecutor
{
    public:
        CarrierUnload()
        : BDIActionExecutor("carrier_unload", 3, false)
        {}

        float advanceWork()
        {
            return 0.112f;            
        }

    private:
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<CarrierUnload>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}