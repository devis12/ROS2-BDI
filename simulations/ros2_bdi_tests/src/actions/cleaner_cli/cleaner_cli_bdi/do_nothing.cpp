#include <string>
#include <vector>

#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class DoNothing : public BDIActionExecutor
{
    public:
        DoNothing()
        : BDIActionExecutor("do_nothing", 1)
        {
        }

        float advanceWork()
        {
            return 0.25f;       
        }

    private:
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<DoNothing>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
