#ifndef SENSOR_H_
#define SENSOR_H_

#include <cstdlib>
#include <ctime>
#include <memory>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "rclcpp/rclcpp.hpp"

typedef enum {ADD, UPD, DEL, NOP} UpdOperation;

class Sensor : public rclcpp::Node
{
public:

  /*
    @sensor_name for the specific name of the node, 
    while @proto_belief defines the prototype for the accepted sensed belief operations performed by the sensor node
    Specifically:
      - proto_belief.pddl_type == 1 (INSTANCE) -> can sense just instances of the same type, 
          i.e. params[0] has to remain always the same, given the fact that it's the type of the instance

      - proto_belief.pddl_type == 2 (PREDICATE) -> can sense exclusively predicates of a given type, 
          i.e. name && params.size() has to remain always the same
      
      - proto_belief.pddl_type == 3 (FUNCTION) -> can sense exclusively function wrt. same instances, 
          i.e. name && params (size, but also individual instances) has to remain always the same, only value changes
  */
  Sensor(const std::string& sensor_name, const ros2_bdi_interfaces::msg::Belief& proto_belief);

  /*  Get belief prototype for the sensor node*/
  ros2_bdi_interfaces::msg::Belief getBeliefPrototype() {return proto_belief_;}
protected:

    /* Sensor logic to be implemented in the actual sensor classes written by the framework user */
    virtual void performSensing() = 0;
    
    /*
        API offered to user so that he can just invoke it within the performSensing() implementation whenever the logic
        requires to update the belief set in some way, i.e. by adding/updating/removing a new belief
    */
    void sense(const ros2_bdi_interfaces::msg::Belief& belief, const UpdOperation& op);

private:

    /*
        Init to call at the start, after construction method, to get the node actually started
        Main thing to be added: frequency at which to perform sensing method which publish the
        result in a belief structure after every sensing
    */
    void init();


    /*
        Called after the initial timeout sleep if set to start the main loop of performSensing() call,
        otherwise the latter is immediately triggered
    */
    void startSensing();

    /*
      Called within the sense method iff the sensed belief is compliant wrt. to the inially
      defined belief prototype in the constructor
    */
    void publishSensing(const UpdOperation& op);

    /*
      perform update if the new instance is valid with respect to the expected sensed one specified in the constructor

      SAME type as the prototype (marked within the first and only param in the params array of the prototype)
    */
    bool sensedInstance(const ros2_bdi_interfaces::msg::Belief& new_belief);

    /*
        perform update if the new predicate is valid with respect to the expected sensed one specified in the constructor

        SAME name as the prototype
        PARAMS number remain fixed (otherwise it's another predicate), but individually they do not need to remain the same
    */
    bool sensedPredicate(const ros2_bdi_interfaces::msg::Belief& new_belief);

    /*
        perform update if the new predicate is valid with respect to the expected sensed one specified in the constructor

        SAME name and exact same params as the prototype
        JUST VALUE can differ
    */
    bool sensedFunction(const ros2_bdi_interfaces::msg::Belief& new_belief);

    // agent id that defines the namespace in which the node operates
    std::string agent_id_;

    // name of the sensor
    std::string sensor_name_;

    // proto_belief_
    // could be:
    //  - just instance type (so the name is the property "added" by the sensor)
    //  - predicate type + name of the predicate (so the args are the properties "added" by the sensor)
    //  - function type + args already defined (so the value is the property "added" by the sensor)
    ros2_bdi_interfaces::msg::Belief proto_belief_;

    // last sensed belief (to be added/deleted/updated)
    ros2_bdi_interfaces::msg::Belief last_sensed_;
    // last sense operation
    UpdOperation last_sensed_op_;

    // callback to perform main loop of work regularly
    rclcpp::TimerBase::SharedPtr sensor_timer_;
    // timer to call one time -> to activate the main loop of sensing (maybe later)
    rclcpp::TimerBase::SharedPtr start_timer_;

    // ros2 publisher to perform publish to topic agent_id_/add_belief, when sense requires it
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr add_belief_publisher_;
    // ros2 publisher to perform publish to topic agent_id_/del_belief, when sense requires it
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr del_belief_publisher_;
};

#endif  // SENSOR_H_