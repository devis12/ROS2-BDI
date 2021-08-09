#include "ros2_bdi_core/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class DoClean : public BDIActionExecutor
{
    public:
        DoClean()
        : BDIActionExecutor("doclean", 4)
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
  
  auto actionNode = std::make_shared<DoClean>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
