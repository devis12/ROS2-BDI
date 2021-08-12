#include "ros2_bdi_core/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class DoSweep : public BDIActionExecutor
{
    public:
        DoSweep()
        : BDIActionExecutor("dosweep", 4)
        {}

        float advanceWork()
        {
            return 0.0625f;            
        }

    private:
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<DoSweep>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
