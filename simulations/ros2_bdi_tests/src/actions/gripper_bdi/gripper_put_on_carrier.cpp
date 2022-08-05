#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class GripperPutOnCarrier : public BDIActionExecutor
{
    public:
        GripperPutOnCarrier()
        : BDIActionExecutor("gripper_put_on_carrier", 4)
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
  
  auto actionNode = std::make_shared<GripperPutOnCarrier>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}