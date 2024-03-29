#ifndef SENSOR_H_
#define SENSOR_H_

#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>
#include <map>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
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
          i.e. type has to remain always the same, given the fact that it's the type of the instance

      - proto_belief.pddl_type == 2 (PREDICATE) -> can sense exclusively predicates of a given type, 
          i.e. name && params.size() has to remain always the same
      
      - proto_belief.pddl_type == 3 (FUNCTION) -> can sense exclusively function wrt. same instances, 
          i.e. name && params (size, but also individual instances) has to remain always the same, only value changes
    
    @match_param_by_param defines whether in a function/fluent sensing all params should match or not (default behaviour is set to true
    to match the rule defined above, but you can override it so that fluent/function case is basically equivalent to the one of the predicate proto.)

    @enable_perform_sensing -> enable periodic perform sensing, i.e. sense is called just within subscription callbacks (default is set to true)
  */
  Sensor(const std::string& sensor_name, const ros2_bdi_interfaces::msg::Belief& proto_belief, 
            const bool match_param_by_param=true, const bool enable_perform_sensing=true);
  
  Sensor(const std::string& sensor_name, const std::vector<ros2_bdi_interfaces::msg::Belief>& proto_belief, 
            const bool match_param_by_param=true, const bool enable_perform_sensing=true);

  /*  Get belief prototype for the sensor node*/
  std::optional<ros2_bdi_interfaces::msg::Belief> getBeliefPrototype(const std::string& name_or_type) 
  {
    if(proto_belief_map_.find(name_or_type) != proto_belief_map_.end())
        return proto_belief_map_[name_or_type];
    else
        return {};
  }
protected:

    /* Sensor logic to be implemented in the actual sensor classes written by the framework user */
    virtual void performSensing() {};
    
    /*
        API offered to user so that he can just invoke it within the performSensing() implementation whenever the logic
        requires to update the belief set in some way, i.e. by adding/updating/removing a new belief
    */
    void sense(const ros2_bdi_interfaces::msg::Belief& belief, const UpdOperation& op);

    /*
        API offered to user so that he can just invoke it within the performSensing() implementation whenever the logic
        requires to update the belief set in some way, i.e. by adding/updating/removing a new belief
    */
    void senseAll(const ros2_bdi_interfaces::msg::BeliefSet& belief_set, const UpdOperation& op);

private:

    /*
        Init to call at the start, after construction method, to get the node actually started
        Main thing to be added: frequency at which to perform sensing method which publish the
        result in a belief structure after every sensing
    */
    void initSensing(const std::string& sensor_name);

    /*
        Init prototype belief set map
    */
    void initPrototypeBsetMap(const std::vector<ros2_bdi_interfaces::msg::Belief>& proto_beliefs);


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

    // applicable with fluent: check param by param equivalence between belief proto and sensed belief
    bool match_param_by_param_;

    // enable performing of periodic sensing 
    bool enable_perform_sensing_;

    // name of the sensor
    std::string sensor_name_;

    // proto_belief_
    // could be:
    //  - just instance type (so the name is the property "added" by the sensor)
    //  - predicate type + name of the predicate (so the args are the properties "added" by the sensor)
    //  - function type + args already defined (so the value is the property "added" by the sensor)
    std::map<std::string, ros2_bdi_interfaces::msg::Belief> proto_belief_map_;

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
    // ros2 publisher to perform publish to topic agent_id_/add_belief_set, when sense requires it
    rclcpp::Publisher<ros2_bdi_interfaces::msg::BeliefSet>::SharedPtr add_belief_set_publisher_;
    // ros2 publisher to perform publish to topic agent_id_/del_belief, when sense requires it
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr del_belief_publisher_;
    // ros2 publisher to perform publish to topic agent_id_/del_belief_set, when sense requires it
    rclcpp::Publisher<ros2_bdi_interfaces::msg::BeliefSet>::SharedPtr del_belief_set_publisher_;
};

#endif  // SENSOR_H_