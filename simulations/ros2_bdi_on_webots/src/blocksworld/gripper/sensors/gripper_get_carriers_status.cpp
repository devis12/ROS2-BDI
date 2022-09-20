#include <memory>
#include <vector>
#include <map>
#include <boost/algorithm/string.hpp>

#include "ros2_bdi_skills/sensor.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "rclcpp/rclcpp.hpp"

#include "webots_ros2_simulations_interfaces/msg/move_status.hpp"

using std::string;
using std::vector;
using std::map;
using std::shared_ptr;
using std::bind;

using ros2_bdi_interfaces::msg::Belief;     
using BDIManaged::ManagedParam;
using BDIManaged::ManagedBelief;


using webots_ros2_simulations_interfaces::msg::MoveStatus;       

const vector<string> id_carriers = {"carrier_a", "carrier_b", "carrier_c"};
const vector<string> id_bases = {"base_a", "base_b", "base_c"};
const vector<string> id_deposits = {"deposit_a", "deposit_b", "deposit_c"};

class CarrierInDepositSensor : public Sensor
{
    public:
        CarrierInDepositSensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            for(uint8_t i = 0; i < id_carriers.size(); i++)
                carriers_move_status_subscriber_.push_back(this->create_subscription<MoveStatus>("/"+id_carriers[i]+"/move_status", 
                    rclcpp::QoS(5).best_effort(),
                    [&, i](const MoveStatus::SharedPtr msg)
                        {
                            std::optional<Belief> bproto = getBeliefPrototype("carrier_in_deposit");
                            if(bproto.has_value())
                            {
                                Belief b = bproto.value();
                                b.params = {id_carriers[i], id_deposits[i]};
                                bool in_dep = boost::algorithm::contains(msg->current_name, "deposit");
                                if(carriers_in_dep_status_.find(id_carriers[i]) == carriers_in_dep_status_.end() || carriers_in_dep_status_[id_carriers[i]] != in_dep)
                                {
                                    //call sense just when the value hasn't been recorded a single time yet or changes    
                                    if (in_dep)
                                        sense(b, UpdOperation::ADD);
                                    else
                                        sense(b, UpdOperation::DEL);
                                    carriers_in_dep_status_[id_carriers[i]] = in_dep;
                                }
                            }
                        }));
        }

    private:
        string robot_name_;
        map<std::string, bool> carriers_in_dep_status_;
        vector<rclcpp::Subscription<MoveStatus>::SharedPtr> carriers_move_status_subscriber_;
};

class CarrierInBaseSensor : public Sensor
{
    public:
        CarrierInBaseSensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            for(uint8_t i = 0; i < id_carriers.size(); i++)
                carriers_move_status_subscriber_.push_back(this->create_subscription<MoveStatus>("/"+id_carriers[i]+"/move_status", 
                    rclcpp::QoS(5).best_effort(),
                    [&, i](const MoveStatus::SharedPtr msg)
                        {
                            std::optional<Belief> bproto = getBeliefPrototype("carrier_in_base");
                            if(bproto.has_value())
                            {
                                Belief b = bproto.value();
                                b.params = {id_carriers[i], id_bases[i]};
                                bool in_base = boost::algorithm::contains(msg->current_name, "base");
                                if(carriers_in_base_status_.find(id_carriers[i]) == carriers_in_base_status_.end() || carriers_in_base_status_[id_carriers[i]] != in_base)
                                {
                                    //call sense just when the value hasn't been recorded a single time yet or changes    
                                    if (in_base)
                                        sense(b, UpdOperation::ADD);
                                    else
                                        sense(b, UpdOperation::DEL);
                                    carriers_in_base_status_[id_carriers[i]] = in_base;
                                }
                            }
                            
                        }));
        }

    private:
        string robot_name_;
        map<std::string, bool> carriers_in_base_status_;
        vector<rclcpp::Subscription<MoveStatus>::SharedPtr> carriers_move_status_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = rclcpp::executors::SingleThreadedExecutor();

  Belief b_proto_carr_in_dep = (ManagedBelief::buildMBPredicate("carrier_in_deposit", {ManagedParam{"?c","carrier"}, ManagedParam{"?dep","deposit"}})).toBelief();
  auto node_carr_dep = std::make_shared<CarrierInDepositSensor>("carrier_in_deposit_sensor", b_proto_carr_in_dep);
  
  Belief b_proto_carr_in_base = (ManagedBelief::buildMBPredicate("carrier_in_base", {ManagedParam{"?c","carrier"}, ManagedParam{"?sb","stackbase"}})).toBelief();
  auto node_carr_base = std::make_shared<CarrierInBaseSensor>("carrier_in_base_sensor", b_proto_carr_in_base);
  
  executor.add_node(node_carr_dep);
  executor.add_node(node_carr_base);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
