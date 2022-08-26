#include <memory>
#include <vector>

#include "ros2_bdi_skills/sensor.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "rclcpp/rclcpp.hpp"

#include "webots_ros2_simulations_interfaces/msg/move_status.hpp"

using std::string;
using std::shared_ptr;
using std::bind;
using std::placeholders::_1;

using ros2_bdi_interfaces::msg::Belief;    
using webots_ros2_simulations_interfaces::msg::MoveStatus;        

using BDIManaged::ManagedParam;
using BDIManaged::ManagedBelief;

class CarrierWPSensor : public Sensor
{
    public:
        CarrierWPSensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief, true, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            carrier_move_status_subscriber_ = this->create_subscription<MoveStatus>("/"+robot_name_+"/move_status", 
                rclcpp::QoS(5).best_effort(),
                std::bind(&CarrierWPSensor::carrierMoveStatusCallback, this, _1));
            
            current_wp_ = Belief();
            current_wp_.name = proto_belief.name;
            current_wp_.pddl_type = proto_belief.pddl_type;
            current_wp_.params = proto_belief.params;
            current_wp_.params[0] = robot_name_;
            current_wp_.params[1] = "deposit";
        }
        
    private:
        void carrierMoveStatusCallback(const MoveStatus::SharedPtr msg)
        {
            move_status_ = *msg;
            if(move_status_.current_name != current_wp_.params[1])
            {    
                sense(current_wp_, DEL);
                current_wp_.params[1] = move_status_.current_name;
                sense(current_wp_, ADD);
            }
        }

        string robot_name_;
        Belief current_wp_;
        MoveStatus move_status_;
        rclcpp::Subscription<MoveStatus>::SharedPtr carrier_move_status_subscriber_;
        int counter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Belief b_proto = (ManagedBelief::buildMBPredicate("in", {ManagedParam{"?c", "carrier"},ManagedParam{"?wp", "waypoint"}})).toBelief();
  auto node = std::make_shared<CarrierWPSensor>("wp_sensor", b_proto);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
