#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class Recharge : public BDIActionExecutor
{
    public:
        Recharge()
        : BDIActionExecutor("recharge", 4)
        {}

        float advanceWork()
        {
            return 0.125f;            
        }

    private:
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<Recharge>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
