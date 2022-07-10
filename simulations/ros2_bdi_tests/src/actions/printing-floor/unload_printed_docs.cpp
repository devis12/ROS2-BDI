#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class UnloadPrintedDocs : public BDIActionExecutor
{
    public:
        UnloadPrintedDocs()
        : BDIActionExecutor("unload_printed_docs", 4)
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
  
  auto actionNode = std::make_shared<UnloadPrintedDocs>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
