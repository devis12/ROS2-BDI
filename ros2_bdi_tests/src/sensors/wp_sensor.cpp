#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>

#include "ros2_bdi_core/sensor.hpp"
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

class WPSensor : public Sensor
{
    public:
        WPSensor(const string& sensor_name, const Belief& belief_proto)
        : Sensor(sensor_name, belief_proto)
        {
            // Init fake waypoint that are going to be sensed
            string agent_id = this->get_parameter(PARAM_AGENT_ID).as_string();
            int agent_num = std::stoi(agent_id.substr(5)); //extract num from name agentX, where X is an int
            Belief bedroom_wp =  (ManagedBelief::buildMBInstance("bedroom"+std::to_string(agent_num), "waypoint")).toBelief();
            Belief bathroom_wp =  (ManagedBelief::buildMBInstance("bathroom"+std::to_string(agent_num), "waypoint")).toBelief();
            Belief kitchen_wp =  (ManagedBelief::buildMBInstance("kitchen"+std::to_string(agent_num), "waypoint")).toBelief();
            waypoints.push_back(bedroom_wp);
            waypoints.push_back(bathroom_wp);
            waypoints.push_back(kitchen_wp);

            counter_ = 0;

            this->init();
        }

        void performSensing()
        {
            if(counter_ < waypoints.size())
            {
                this->sense(waypoints[counter_]);
                RCLCPP_INFO(this->get_logger(), "WaypointSensor sensing for instance of type " + this->belief_proto_.params[0] + 
                    " has sensed: " + waypoints[counter_].name);
                counter_++;
            }
            
        }

    private:
        vector<Belief> waypoints;
        int counter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Belief b_proto = (ManagedBelief::buildMBInstance("proto", "waypoint")).toBelief();
  auto node = std::make_shared<WPSensor>("wp_sensor", b_proto);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
