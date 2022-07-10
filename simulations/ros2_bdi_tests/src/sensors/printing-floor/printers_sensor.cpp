#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>

#include "ros2_bdi_skills/sensor.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "rclcpp/rclcpp.hpp"

#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"

using std::string;
using std::vector;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::optional;

using ros2_bdi_interfaces::msg::Belief;            

using BDIManaged::ManagedBelief;

class PrintersSensor : public Sensor
{
    public:
        PrintersSensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief)
        {
            // Init fake printers available beliefs that are going to be sensed
            p1_available_ = (ManagedBelief::buildMBPredicate(proto_belief.name, {"p1"})).toBelief();
            p2_available_ = (ManagedBelief::buildMBPredicate(proto_belief.name, {"p2"})).toBelief();

            this->declare_parameter("p1_available", true);
            this->declare_parameter("p2_available", false);
        }

        void performSensing() override
        {
            sense(p1_available_, this->get_parameter("p1_available").as_bool()? ADD:DEL);
            sense(p2_available_, this->get_parameter("p2_available").as_bool()? ADD:DEL);
        }

    private:
        Belief p1_available_;
        Belief p2_available_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Belief b_proto = (ManagedBelief::buildMBPredicate("available", {""})).toBelief();
  auto node = std::make_shared<PrintersSensor>("printers_sensor", b_proto);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
