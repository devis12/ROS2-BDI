#ifndef SENSOR_H_
#define SENSOR_H_

#include <cstdlib>
#include <ctime>
#include <memory>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "rclcpp/rclcpp.hpp"

#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"
#define PARAM_SENSING_FREQ "sensing_freq" 
#define PARAM_SENSOR_NAME "sensor_name"
#define PARAM_INIT_SLEEP "init_sleep"

typedef enum {ADD, UPD, DEL, NOP} UpdOperation;

class Sensor : public rclcpp::Node
{
public:
  Sensor(const std::string& sensor_name, const ros2_bdi_interfaces::msg::Belief& belief_proto);

protected:
    /*
        Init to call at the start, after construction method, to get the node actually started
        Main thing to be added: frequency at which to perform sensing method which publish the
        result in a belief structure after every sensing
    */
    void init();

    void startSensing();

    virtual void performSensing() = 0;
    
    void sense(const ros2_bdi_interfaces::msg::Belief& new_belief, const UpdOperation& op);

    // name of the sensor
    std::string sensor_name_;
    // pddl type of the sensor
    ros2_bdi_interfaces::msg::Belief belief_proto_;

private:

    /*
      Main loop of publishing called regularly through a wall timer
      publish just if publish set to true
    */
    void publishSensing(const UpdOperation& op);

    /*
      perform update if the new instance is valid with respect to the expected sensed one
    */
    bool sensedInstance(const ros2_bdi_interfaces::msg::Belief& new_belief);

    /*
      perform update if the new predicate is valid with respect to the expected sensed one
    */
    bool sensedPredicate(const ros2_bdi_interfaces::msg::Belief& new_belief);
    /*
      perform update if the new predicate is valid with respect to the expected sensed one
    */
    bool sensedFunction(const ros2_bdi_interfaces::msg::Belief& new_belief);

    // agent id that defines the namespace in which the node operates
    std::string agent_id_;

    // sensed belief
    // could be:
    //  - just instance type (so the name is the property "added" by the sensor)
    //  - predicate type + name of the predicate (so the args are the properties "added" by the sensor)
    //  - function type + args already defined (so the value is the property "added" by the sensor)
    ros2_bdi_interfaces::msg::Belief sensed_belief_;
    UpdOperation sensed_op_;

    // callback to perform main loop of work regularly
    rclcpp::TimerBase::SharedPtr sensor_timer_;
    // timer to call one time -> to activate the main loop of sensing (maybe later)
    rclcpp::TimerBase::SharedPtr start_timer_;

    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr add_belief_publisher_;//add_belief publisher

    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr del_belief_publisher_;//del_belief publisher
};

#endif  // SENSOR_H_