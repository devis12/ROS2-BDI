#include "ros2_bdi_skills/sensor.hpp"
// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Sensor node
#include "ros2_bdi_skills/params/sensor_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node (for belief set topic)
#include "ros2_bdi_core/params/belief_manager_params.hpp"

using std::string;
using std::vector;
using std::map;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::optional;

using ros2_bdi_interfaces::msg::Belief;  
using ros2_bdi_interfaces::msg::BeliefSet;

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
*/
Sensor::Sensor(const string& sensor_name, const Belief& proto_belief, const bool match_param_by_param, const bool enable_perform_sensing) 
    : rclcpp::Node(sensor_name),

    match_param_by_param_(match_param_by_param),
    enable_perform_sensing_(enable_perform_sensing)
{
    this->initPrototypeBsetMap({proto_belief});

    this->initSensing(sensor_name);
}

Sensor::Sensor(const string& sensor_name, const vector<Belief>& proto_beliefs, const bool match_param_by_param, const bool enable_perform_sensing) 
    : rclcpp::Node(sensor_name),

    match_param_by_param_(match_param_by_param),
    enable_perform_sensing_(enable_perform_sensing)
{
    this->initPrototypeBsetMap(proto_beliefs);

    this->initSensing(sensor_name);
}

/*
    Init prototype belief set map
*/
void Sensor::initPrototypeBsetMap(const vector<Belief>& proto_beliefs)
{
    proto_belief_map_= map<string, Belief>();
    Belief proto_belief_checked = Belief();
    last_sensed_op_ = NOP;

    for(Belief proto_belief : proto_beliefs)
    {
        //Erroneous type will generate sensors that do not publish any sensing info
        if(proto_belief.pddl_type == Belief().INSTANCE_TYPE ||
            proto_belief.pddl_type == Belief().PREDICATE_TYPE ||
            proto_belief.pddl_type == Belief().FUNCTION_TYPE
        )
            proto_belief_checked.pddl_type = proto_belief.pddl_type;

        proto_belief_checked.name = proto_belief.name;
        proto_belief_checked.params = proto_belief.params;
        proto_belief_checked.type = proto_belief.type;
        
        if(proto_belief.pddl_type == Belief().INSTANCE_TYPE)
            proto_belief_map_[proto_belief_checked.type] = proto_belief_checked;
        if(proto_belief.pddl_type == Belief().PREDICATE_TYPE || proto_belief.pddl_type == Belief().FUNCTION_TYPE)
            proto_belief_map_[proto_belief_checked.name] = proto_belief_checked;
    }
}

/*
    Init to call at the start, after construction method, to get the node actually started
    Main thing to be added: frequency at which to perform sensing method which publish the
    result in a belief structure after every sensing
*/
void Sensor::initSensing(const string& sensor_name)
{ 
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, false);
    this->declare_parameter(PARAM_SENSOR_NAME, sensor_name);
    this->declare_parameter(PARAM_SENSING_FREQ, 8.0);//sensing frequency by default set to 8Hz
    this->declare_parameter(PARAM_INIT_SLEEP, 2);//init node sleep (e.g. sensor activated later) // default now is 2 to wait for the other to boot as well (since they wait a bit for psys2) 

    // agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    // Add new belief publisher
    add_belief_publisher_ = this->create_publisher<Belief>(ADD_BELIEF_TOPIC, 10);

    // Add set of beliefs publisher
    add_belief_set_publisher_ = this->create_publisher<BeliefSet>(ADD_BELIEF_SET_TOPIC, 10);

    // Del belief publisher
    del_belief_publisher_ = this->create_publisher<Belief>(DEL_BELIEF_TOPIC, 10);

    // Del set of beliefs publisher
    del_belief_set_publisher_ = this->create_publisher<BeliefSet>(DEL_BELIEF_SET_TOPIC, 10);
    
    // retrieve from parameter frequency at which to perform sensing
    float sensing_freq = this->get_parameter(PARAM_SENSING_FREQ).as_double();

    int init_sleep_sec = this->get_parameter(PARAM_INIT_SLEEP).as_int();
    if (init_sleep_sec <= 0)// directly init sensor_timer_

        sensor_timer_ = this->create_wall_timer(
            milliseconds((int) (1000/sensing_freq)),
            bind(&Sensor::performSensing, this));// loop to be called regularly to publish the sensing result (publish add_belief)

    else if(enable_perform_sensing_)// wait init sleep seconds before starting sensor_timer_
        start_timer_ = this->create_wall_timer(
            seconds(init_sleep_sec),
            bind(&Sensor::startSensing, this));

    RCLCPP_INFO(this->get_logger(), "Sensor node \"" + this->get_parameter(PARAM_SENSOR_NAME).as_string() + "\" initialized");
}

/*
    Called after the initial timeout sleep if set to start the main loop of performSensing() call,
    otherwise the latter is immediately triggered
*/
void Sensor::startSensing()
{   
    // cancel start time
    start_timer_->cancel();

    // retrieve from parameter frequency at which to perform sensing
    float sensing_freq = this->get_parameter(PARAM_SENSING_FREQ).as_double();
    if(enable_perform_sensing_)
        sensor_timer_ = this->create_wall_timer(    // loop to be called regularly to publish the sensing result (publish add_belief)
            milliseconds((int) (1000/sensing_freq)),
            bind(&Sensor::performSensing, this));
}

/*
    API offered to user so that he can just invoke it within the performSensing() implementation whenever the logic
    requires to update the belief set in some way, i.e. by adding/updating/removing a new belief
*/
void Sensor::sense(const Belief& belief, const UpdOperation& op)
{
    bool updated = false;
    
    if(proto_belief_map_.find(belief.type) != proto_belief_map_.end() && proto_belief_map_[belief.type].pddl_type == Belief().INSTANCE_TYPE)
        updated = sensedInstance(belief);
    else if(proto_belief_map_.find(belief.name) != proto_belief_map_.end() && proto_belief_map_[belief.name].pddl_type == Belief().PREDICATE_TYPE)
        updated = sensedPredicate(belief);
    else if(proto_belief_map_.find(belief.name) != proto_belief_map_.end() && proto_belief_map_[belief.name].pddl_type == Belief().FUNCTION_TYPE)
        updated = sensedFunction(belief);
    if(updated)
        publishSensing(op);
}



/*
    API offered to user so that he can just invoke it within the performSensing() implementation whenever the logic
    requires to update the belief set in some way, i.e. by adding/updating/removing a new belief
*/
void Sensor::senseAll(const BeliefSet& belief_set, const UpdOperation& op)
{
    BeliefSet filteredBSetMsg = BeliefSet{};
    filteredBSetMsg.agent_id = agent_id_;
    for(Belief belief : belief_set.value)
    {
        bool updated = false;
    
        if(proto_belief_map_.find(belief.type) != proto_belief_map_.end() && proto_belief_map_[belief.type].pddl_type == Belief().INSTANCE_TYPE)
            updated = sensedInstance(belief);
        else if(proto_belief_map_.find(belief.name) != proto_belief_map_.end() && proto_belief_map_[belief.name].pddl_type == Belief().PREDICATE_TYPE)
            updated = sensedPredicate(belief);
        else if(proto_belief_map_.find(belief.name) != proto_belief_map_.end() && proto_belief_map_[belief.name].pddl_type == Belief().FUNCTION_TYPE)
            updated = sensedFunction(belief);
        
        if(updated)
            filteredBSetMsg.value.push_back(belief);
    }

    if(op == ADD || op == UPD)
        add_belief_set_publisher_->publish(filteredBSetMsg);
    else if(op == DEL)
        del_belief_set_publisher_->publish(filteredBSetMsg);
}

/*
    Called within the sense method iff the sensed belief is compliant wrt. to the inially
    defined belief prototype in the constructor
*/
void Sensor::publishSensing(const UpdOperation& op)
{
    last_sensed_op_ = op;
    
    if(op == ADD || op == UPD)
        add_belief_publisher_->publish(last_sensed_);
    else if(op == DEL)
        del_belief_publisher_->publish(last_sensed_);

    if(this->get_parameter(PARAM_DEBUG).as_bool())
    {
        string stringOp = ((op==ADD)? "ADD" : "DEL");
        string paramsJoined = "";
        for(auto p : last_sensed_.params)
            paramsJoined += p + ", ";
        string stringBelief = "name = " + last_sensed_.name + ", params = " + paramsJoined + 
            ((last_sensed_.pddl_type == last_sensed_.FUNCTION_TYPE)? " value = " + std::to_string(last_sensed_.value) : "");
        RCLCPP_INFO(this->get_logger(), "Operation = " + stringOp + " , belief " + stringBelief);
    }
}

/*
    perform update if the new instance is valid with respect to the expected sensed one specified in the constructor

    SAME type as the prototype (marked within the first and only param in the params array of the prototype)
*/
bool Sensor::sensedInstance(const Belief& new_belief)
{   
    bool found = proto_belief_map_.find(new_belief.type) != proto_belief_map_.end();
    bool do_upd = found && new_belief.pddl_type == Belief().INSTANCE_TYPE && proto_belief_map_[new_belief.type].type == new_belief.type;
    if(do_upd) //only 1 param which indicates the type which has to remain the same
        last_sensed_ = new_belief;
    return do_upd;
}

/*
    perform update if the new predicate is valid with respect to the expected sensed one specified in the constructor

    SAME name as the prototype
    PARAMS number remain fixed (otherwise it's another predicate), but individually they do not need to remain the same
*/
bool Sensor::sensedPredicate(const Belief& new_belief)
{
    bool found = proto_belief_map_.find(new_belief.name) != proto_belief_map_.end();
    bool do_upd = found && new_belief.pddl_type == Belief().PREDICATE_TYPE && new_belief.name == proto_belief_map_[new_belief.name].name 
    && new_belief.params.size() == proto_belief_map_[new_belief.name].params.size(); //same name -> same predicate -> same num of params
    
    if(do_upd)
        last_sensed_ = new_belief;
    return do_upd;
}

/*
    perform update if the new predicate is valid with respect to the expected sensed one specified in the constructor

    SAME name and exact same params as the prototype
    JUST VALUE can differ
*/
bool Sensor::sensedFunction(const Belief& new_belief)
{
    bool found = proto_belief_map_.find(new_belief.name) != proto_belief_map_.end();
    bool do_upd = found && new_belief.pddl_type == Belief().FUNCTION_TYPE && new_belief.name == proto_belief_map_[new_belief.name].name 
    && new_belief.params.size() == proto_belief_map_[new_belief.name].params.size(); //same name -> same function -> same params

    if(match_param_by_param_)
    {
        for(int i=0; do_upd && i<proto_belief_map_[new_belief.name].params.size(); i++)//check match param by param
            if(proto_belief_map_[new_belief.name].params != new_belief.params)
                do_upd = false;
    }
    if(do_upd)
        last_sensed_ = new_belief;
    return do_upd;
}