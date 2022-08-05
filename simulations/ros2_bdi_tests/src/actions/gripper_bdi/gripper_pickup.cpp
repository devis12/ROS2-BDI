#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class GripperPickup : public BDIActionExecutor
{
    public:
        GripperPickup()
        : BDIActionExecutor("gripper_pickup", 4)
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
  
  auto actionNode = std::make_shared<GripperPickup>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
