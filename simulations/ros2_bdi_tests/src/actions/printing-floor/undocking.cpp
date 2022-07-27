#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class Undocking : public BDIActionExecutor
{
    public:
        Undocking()
        : BDIActionExecutor("undocking", 2)
        {}

        float advanceWork()
        {
            return 0.084f;            
        }

    private:
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<Undocking>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
