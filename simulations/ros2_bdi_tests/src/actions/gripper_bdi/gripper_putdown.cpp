#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class GripperPutdown : public BDIActionExecutor
{
    public:
        GripperPutdown()
        : BDIActionExecutor("gripper_putdown", 4)
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
  
  auto actionNode = std::make_shared<GripperPutdown>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}