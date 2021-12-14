#include "ros2_bdi_skills/sensor.hpp"

using std::string;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::optional;

using ros2_bdi_interfaces::msg::Belief;  

Sensor::Sensor(const string& sensor_name, const Belief& belief_proto) : rclcpp::Node(sensor_name)
{
    sensed_belief_ = Belief{};
    sensed_op_ = NOP;

    //Erroneous type will generate sensors that do not publish any sensing info
    if(belief_proto.pddl_type == Belief().INSTANCE_TYPE)
        sensed_belief_.pddl_type = Belief().INSTANCE_TYPE;
    else if(belief_proto.pddl_type == Belief().PREDICATE_TYPE)
        sensed_belief_.pddl_type = Belief().PREDICATE_TYPE;
    else if(belief_proto.pddl_type == Belief().FUNCTION_TYPE)
        sensed_belief_.pddl_type = Belief().FUNCTION_TYPE;

    sensed_belief_.name = belief_proto.name;
    sensed_belief_.params = belief_proto.params;

    belief_proto_ = belief_proto;
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);
    this->declare_parameter(PARAM_SENSOR_NAME, sensor_name);
    this->declare_parameter(PARAM_SENSING_FREQ, 8.0);//sensing frequency by default set to 8Hz
    this->declare_parameter(PARAM_INIT_SLEEP, 0);//init node sleep (e.g. sensor activated later)
}


void Sensor::init()
{ 
    // agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    // Add new belief publisher
    add_belief_publisher_ = this->create_publisher<Belief>("add_belief", 10);

    // Del belief publisher
    del_belief_publisher_ = this->create_publisher<Belief>("del_belief", 10);
    
    // retrieve from parameter frequency at which to perform sensing
    float sensing_freq = this->get_parameter(PARAM_SENSING_FREQ).as_double();

    int init_sleep_sec = this->get_parameter(PARAM_INIT_SLEEP).as_int();
    if (init_sleep_sec <= 0)// directly init sensor_timer_

        sensor_timer_ = this->create_wall_timer(
            milliseconds((int) (1000/sensing_freq)),
            bind(&Sensor::performSensing, this));// loop to be called regularly to publish the sensing result (publish add_belief)

    else// wait init sleep seconds before starting sensor_timer_
        start_timer_ = this->create_wall_timer(
            seconds(init_sleep_sec),
            bind(&Sensor::startSensing, this));

    RCLCPP_INFO(this->get_logger(), "Sensor node \"" + this->get_parameter(PARAM_SENSOR_NAME).as_string() + "\" initialized");
}

void Sensor::startSensing()
{   
    // cancel start time
    start_timer_->cancel();

    // retrieve from parameter frequency at which to perform sensing
    float sensing_freq = this->get_parameter(PARAM_SENSING_FREQ).as_double();
    // loop to be called regularly to publish the sensing result (publish add_belief)
    sensor_timer_ = this->create_wall_timer(
        milliseconds((int) (1000/sensing_freq)),
        bind(&Sensor::performSensing, this));
}
    
void Sensor::sense(const Belief& new_belief, const UpdOperation& op)
{
    if(sensed_belief_.pddl_type != new_belief.pddl_type)//sensing not valid 
        return;
    
    bool updated = false;

    if(sensed_belief_.pddl_type == Belief().INSTANCE_TYPE)
        updated = sensedInstance(new_belief);
    else if(sensed_belief_.pddl_type == Belief().PREDICATE_TYPE)
        updated = sensedPredicate(new_belief);
    else if(sensed_belief_.pddl_type == Belief().FUNCTION_TYPE)
        updated = sensedFunction(new_belief);

    if(updated)
        publishSensing(op);
}

/*
    Main loop of publishing called regularly through a wall timer
    publish just if publish set to true
*/
void Sensor::publishSensing(const UpdOperation& op)
{
    sensed_op_ = op;
    
    if(op == ADD || op == UPD)
    add_belief_publisher_->publish(sensed_belief_);
    else if(op == DEL)
    del_belief_publisher_->publish(sensed_belief_);
}

/*
    perform update if the new instance is valid with respect to the expected sensed one
*/
bool Sensor::sensedInstance(const Belief& new_belief)
{
    bool do_upd = new_belief.pddl_type == Belief().INSTANCE_TYPE && new_belief.params.size() == 1  //only 1 param which indicates the type
    && sensed_belief_.params[0] == new_belief.params[0];
    if(do_upd) //only 1 param which indicates the type
    sensed_belief_ = new_belief;
    return do_upd;
}

/*
    perform update if the new predicate is valid with respect to the expected sensed one
*/
bool Sensor::sensedPredicate(const Belief& new_belief)
{
    bool do_upd = new_belief.pddl_type == Belief().PREDICATE_TYPE && new_belief.name == sensed_belief_.name 
    && new_belief.params.size() == sensed_belief_.params.size(); //same name -> same predicate -> same num of params
    if(do_upd)
    sensed_belief_ = new_belief;
    return do_upd;
}

/*
    perform update if the new predicate is valid with respect to the expected sensed one
*/
bool Sensor::sensedFunction(const Belief& new_belief)
{
    bool do_upd = new_belief.pddl_type == Belief().FUNCTION_TYPE && new_belief.name == sensed_belief_.name 
    && new_belief.params.size() == sensed_belief_.params.size(); //same name -> same function -> same params

    for(int i=0; do_upd && i<sensed_belief_.params.size(); i++)//check match param by param
    if(sensed_belief_.params != new_belief.params)
        do_upd = false;
    
    if(do_upd)
    sensed_belief_ = new_belief;
    return do_upd;
}