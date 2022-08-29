// header file for Belief Manager node
#include "ros2_bdi_core/belief_manager.hpp"
// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node
#include "ros2_bdi_core/params/belief_manager_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for PlanSys2 Monitor node (for psys2 state topic)
#include "ros2_bdi_core/params/plansys_monitor_params.hpp"

#include <yaml-cpp/exceptions.h>

#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "ros2_bdi_utils/BDIPDDLConverter.hpp"
#include "ros2_bdi_utils/PDDLBDIConverter.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"
#include "ros2_bdi_utils/BDIYAMLParser.hpp"


using std::string;
using std::vector;
using std::set;
using std::map;
using std::mutex;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;

using plansys2::ProblemExpertClient;
using plansys2::DomainExpertClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;
using plansys2_msgs::srv::GetProblemInstances;
using std_msgs::msg::Empty;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::LifecycleStatus;
using ros2_bdi_interfaces::msg::PlanningSystemState;

using BDIManaged::ManagedParam;
using BDIManaged::ManagedBelief;


/*  Constructor method */
BeliefManager::BeliefManager()
  : rclcpp::Node(BELIEF_MANAGER_NODE_NAME), state_(STARTING)
{
    psys2_comm_errors_ = 0;
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);
    this->declare_parameter(PARAM_PLANNING_MODE, PLANNING_MODE_OFFLINE);

    sel_planning_mode_ = this->get_parameter(PARAM_PLANNING_MODE).as_string() == PLANNING_MODE_OFFLINE? OFFLINE : ONLINE;
    this->undeclare_parameter(PARAM_PLANNING_MODE);
}


/*
    Init to call at the start, after construction method, to get the node actually started
    initialing problem_expert_ instance, 
    retrieving agent_id_ (thus namespace)
    defining work timer,
    belief set publishers/subscribers callback,
    problem_expert_update notification
*/
void BeliefManager::init()
{ 
    //agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    //domain expert client to communicate with domain expert node of plansys2
    domain_expert_ = std::make_shared<DomainExpertClient>();

    //problem expert client to communicate with problem expert node of plansys2
    problem_expert_ = std::make_shared<ProblemExpertClient>();

    // last pddl problem known at the moment init (just empty string)
    last_pddl_problem_ = "";

    //Declare empty belief set
    belief_set_ = set<ManagedBelief>();
    //wait for it to be init
    init_bset_ = false;

    //Belief set publisher
    belief_set_publisher_ = this->create_publisher<BeliefSet>(BELIEF_SET_TOPIC, 10);
    
    rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
    qos_keep_all.keep_all();

    //lifecycle status init
    auto lifecycle_status = LifecycleStatus{};
    lifecycle_status_ = map<string, uint8_t>();
    lifecycle_status_[BELIEF_MANAGER_NODE_NAME] = lifecycle_status.BOOTING;
    lifecycle_status_[SCHEDULER_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[PLAN_DIRECTOR_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[PSYS_MONITOR_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[EVENT_LISTENER_NODE_NAME] = lifecycle_status.UNKNOWN;
    lifecycle_status_[MA_REQUEST_HANDLER_NODE_NAME] = lifecycle_status.UNKNOWN;

    // init step_counter
    step_counter_ = 0;

    //Lifecycle status publisher
    lifecycle_status_publisher_ = this->create_publisher<LifecycleStatus>(LIFECYCLE_STATUS_TOPIC, 10);

    //Lifecycle status subscriber
    lifecycle_status_subscriber_ = this->create_subscription<LifecycleStatus>(
                LIFECYCLE_STATUS_TOPIC, qos_keep_all,
                bind(&BeliefManager::callbackLifecycleStatus, this, _1));

    //Check for plansys2 active state flags init to false
    psys2_domain_expert_active_ = false;
    psys2_problem_expert_active_ = false;

    //plansys2 nodes status subscriber (receive notification from plansys2_monitor node)
    plansys2_status_subscriber_ = this->create_subscription<PlanningSystemState>(
                PSYS_STATE_TOPIC, qos_keep_all,
                bind(&BeliefManager::callbackPsys2State, this, _1));

    //Belief to be added notification
    add_belief_subscriber_ = this->create_subscription<Belief>(
                ADD_BELIEF_TOPIC, qos_keep_all,
                bind(&BeliefManager::addBeliefTopicCallBack, this, _1));
    //Belief to be removed notification
    del_belief_subscriber_ = this->create_subscription<Belief>(
                DEL_BELIEF_TOPIC, qos_keep_all,
                bind(&BeliefManager::delBeliefTopicCallBack, this, _1));

    //problem_expert update subscriber
    updated_problem_subscriber_ = this->create_subscription<Empty>(
                "problem_expert/update_notify", 10,
                bind(&BeliefManager::updatedPDDLProblem, this, _1));

    //loop to be called regularly to perform work (publish belief_set_, sync with plansys2 problem_expert node...)
    do_work_timer_ = this->create_wall_timer(
        milliseconds(500),
        bind(&BeliefManager::step, this));

    RCLCPP_INFO(this->get_logger(), "Belief manager node initialized");
}
  
/*
    Main loop of work called regularly through a wall timer
*/
void BeliefManager::step()
{
    // all psys2 up -> no psys2 comm. errors
    if(psys2_domain_expert_active_ && psys2_problem_expert_active_)
        psys2_comm_errors_ = 0;

    //if psys2 appears crashed, crash too
    if(psys2_comm_errors_ > MAX_COMM_ERRORS)
        rclcpp::shutdown();

    if(step_counter_ % 4 == 0)
        lifecycle_status_publisher_->publish(getLifecycleStatus());

    switch (state_) {
        
        case STARTING:
        {
            if(psys2_domain_expert_active_ && psys2_problem_expert_active_ ){
                psys2_comm_errors_ = 0;
                if(!init_bset_)//hasn't been tried to init belief set yet
                {    
                    tryInitBeliefSet();
                    init_bset_ = true;
                }
                setState(SYNC);
                lifecycle_status_publisher_->publish(getLifecycleStatus());
            }else{
                if(!psys2_domain_expert_active_)
                    RCLCPP_ERROR(this->get_logger(), "PlanSys2 Domain Expert still not active");
                if(!psys2_problem_expert_active_)
                    RCLCPP_ERROR(this->get_logger(), "PlanSys2 Problem Expert still not active");
                psys2_comm_errors_++;
            }

            break;
        }

        case SYNC:
        {    
            publishBeliefSet();
        }

        case PAUSE:
        {   
            //RCLCPP_INFO(this->get_logger(), "Not the moment to ask for new cleaning tasks yet");    
            break;
        }
        default:
            break;
    }

    step_counter_++;
}

/*Build updated LifecycleStatus msg*/
LifecycleStatus BeliefManager::getLifecycleStatus()
{
    LifecycleStatus lifecycle_status = LifecycleStatus{};
    lifecycle_status.node_name = BELIEF_MANAGER_NODE_NAME;
    lifecycle_status.status = (state_ == SYNC || state_ == PAUSE)? lifecycle_status.RUNNING : lifecycle_status.BOOTING;
    return lifecycle_status;
}


/*
    Received notification about PlanSys2 nodes state by plansys2 monitor node
*/
void BeliefManager::callbackPsys2State(const PlanningSystemState::SharedPtr msg)
{
    psys2_problem_expert_active_ = msg->problem_expert_active;
    psys2_domain_expert_active_ = msg->domain_expert_active;
}

/*
    Publish the current belief set of the agent in agent_id_/belief_set topic
*/
void BeliefManager::publishBeliefSet()
{
    BeliefSet bset_msg = BDIFilter::extractBeliefSetMsg(belief_set_);
    bset_msg.agent_id = agent_id_;
    belief_set_publisher_->publish(bset_msg);
}

/*
    Expect to find yaml file to init the belief set in "/tmp/{agent_id}/init_bset.yaml"
*/
void BeliefManager::tryInitBeliefSet()
{
    string init_bset_filepath = "/tmp/"+this->get_parameter(PARAM_AGENT_ID).as_string()+"/"+INIT_BELIEF_SET_FILENAME;
    
    try{
        vector<ManagedBelief> init_mgbeliefs = BDIYAMLParser::extractMGBeliefs(init_bset_filepath, domain_expert_);
        for(ManagedBelief initMGBelief : init_mgbeliefs)
            addBeliefSyncPDDL(initMGBelief);
        if(this->get_parameter(PARAM_DEBUG).as_bool())
            RCLCPP_INFO(this->get_logger(), "Belief set initialization performed through " + init_bset_filepath);
    
    }catch(const YAML::BadFile& bfile){
        RCLCPP_ERROR(this->get_logger(), "Bad File: Belief set initialization failed because init. file " + init_bset_filepath + " hasn't been found");
    }catch(const YAML::ParserException& bpars){
        RCLCPP_ERROR(this->get_logger(), "YAML Parser Exception: Belief set initialization failed because init. file " + init_bset_filepath + " doesn't present a valid YAML format");
    }catch(const YAML::BadConversion& bconvfile){
        RCLCPP_ERROR(this->get_logger(), "Bad Conversion: Belief set initialization failed because init. file " + init_bset_filepath + " doesn't present a valid belief array");
    }catch(const YAML::InvalidNode& invalid_node){
        RCLCPP_ERROR(this->get_logger(), "Invalid Node: Belief set initialization failed because init. file " + init_bset_filepath + " doesn't present a valid belief array");
    }
}

/*
    Callback wrt. "problem_expert/update_notify" topic which notifies about any change in the PDDL problem
    update belief set accordingly
*/
void BeliefManager::updatedPDDLProblem(const Empty::SharedPtr msg)
{   
    string pddlProblemNow = problem_expert_->getProblem();
    //strip off goal part (the belief regards just instances, predicates, fluents)
    pddlProblemNow = pddlProblemNow.substr(0,pddlProblemNow.find(":goal")-1);
    if(pddlProblemNow == last_pddl_problem_)//nothing has changed (maybe goal has been set)
        return;

    last_pddl_problem_ = pddlProblemNow;
    bool notify;//if anything changes, put it to true
            
    if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_INFO(this->get_logger(), "Update pddl problem notification:\n"+pddlProblemNow);

    vector<Belief> instances = PDDLBDIConverter::convertPDDLInstances(problem_expert_->getInstances());
    vector<Belief> predicates = PDDLBDIConverter::convertPDDLPredicates(problem_expert_->getPredicates());
    vector<Belief> functions = PDDLBDIConverter::convertPDDLFunctions(problem_expert_->getFunctions());
    notify = updateBeliefSet(instances, predicates, functions);
    
    if(notify)
        publishBeliefSet();//there has been some modifications, publish new belief set
}


bool BeliefManager::updateBeliefSet(const vector<Belief>& ins_beliefs, const vector<Belief>& pred_beliefs, const vector<Belief>& fun_beliefs)
{
    bool notify;//if anything changes, put it to true
    if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_INFO(this->get_logger(), "update problem: verify if needed to sync (b_set %d, prob_ins %d, prob_pred %d, prob_fun %d)", 
            belief_set_.size(), ins_beliefs.size(), pred_beliefs.size(), fun_beliefs.size());

    mtx_sync.lock();
        //try to add new beliefs or modify current beliefs 
        notify = (ins_beliefs.size() > 0 && addOrModifyBeliefs(ins_beliefs, false));
        notify = (pred_beliefs.size() > 0 && addOrModifyBeliefs(pred_beliefs, false)) || notify;
        notify = (fun_beliefs.size() > 0 && addOrModifyBeliefs(fun_beliefs, true)) || notify;

        //check for the removal of some beliefs
        notify = (belief_set_.size() > 0 && removedInstanceBeliefs(ins_beliefs)) || notify;
        notify = (belief_set_.size() > 0 && removedPredicateBeliefs(pred_beliefs)) || notify;
        notify = (belief_set_.size() > 0 && removedFunctionBeliefs(fun_beliefs)) || notify;
    mtx_sync.unlock();

    return notify;//there has been some modifications
}

    /*
    Update belief_set checking for presence/absence/alteration of some belief in it wrt. predicates/function
    retrieved from the problem_expert
    Returns true if any modification to the belief_set occurs

    @check_for_function put to true when you want to check also for modified functions wrt. their values
*/
bool BeliefManager::addOrModifyBeliefs(const vector<Belief>& beliefs, const bool check_for_function)
{
    bool modified = false;//if anything changes, put it to true

    //check for new or modified (just in case of function type) beliefs
    for(auto bel : beliefs)
    {   
        ManagedBelief mb = ManagedBelief{bel};
        int count_bs = belief_set_.count(mb);
        
        if(count_bs == 0)//not found
        {
            RCLCPP_INFO(this->get_logger(), "Adding missing belief ("+mb.pddlTypeString()+"): " + 
                mb.getName() + " " + ((mb.pddlType() == Belief().INSTANCE_TYPE)? mb.type() : mb.getParamsJoined()) +
                ((mb.pddlType() == Belief().FUNCTION_TYPE)?
                    " (value = " + std::to_string(mb.getValue()) +")" : "")
                );
            
            addBelief(bel);
            modified = true;//there is an alteration, notify it
        }
        else if(check_for_function && bel.value != (*(belief_set_.find(mb))).getValue())
        {
            modifyBelief(mb);
            modified = true;//there is an alteration, notify it
        }
    }

    return modified;
}

/*
    Extract from the passed beliefs and from the belief set just beliefs of type instance and check if something
    is in the beliefs array and not in the belief set. If so, remove it from the belief set and notify there has
    been any alteration through the boolean value in return.
*/
bool BeliefManager::removedInstanceBeliefs(const vector<Belief>& beliefs)
{
    bool modified = false;//if anything changes, put it to true

    set<ManagedBelief> instances_in_belief_set = BDIFilter::extractMGInstances(belief_set_);
    set<ManagedBelief> instances_in_pddl_prob = BDIFilter::extractMGInstances(beliefs);

    if(instances_in_belief_set.size() > instances_in_pddl_prob.size())
    {
        //some instance has to be removed from the belief set, since it has already been removed from the pddl problem
        for(ManagedBelief mb : instances_in_belief_set)
            if(instances_in_pddl_prob.count(mb) == 0)
            {
                modified = true;
                delBelief(mb);//mb shall be removed from belief_set_ since it's not present in the pddl_problem
            }
    }

    return modified;
}

/*
    Extract from the passed beliefs and from the belief set just beliefs of type function and check if something
    is in the beliefs array and not in the belief set. If so, remove it from the belief set and notify there has
    been any alteration through the boolean value in return.
*/
bool BeliefManager::removedFunctionBeliefs(const vector<Belief>& beliefs)
{
    bool modified = false;//if anything changes, put it to true

    set<ManagedBelief> function_in_belief_set = BDIFilter::extractMGFunctions(belief_set_);
    set<ManagedBelief> function_in_pddl_prob = BDIFilter::extractMGFunctions(beliefs);

    if(function_in_belief_set.size() > function_in_pddl_prob.size())
    {
        //some function has to be removed from the belief set, since it has already been removed from the pddl problem
        for(ManagedBelief mb : function_in_belief_set)
            if(function_in_pddl_prob.count(mb) == 0)
            {
                modified = true;
                delBelief(mb);//mb shall be removed from belief_set_ since it's not present in the pddl_problem
            }
    }

    return modified;
}

/*
    Extract from the passed beliefs and from the belief set just beliefs of type predicate and check if something
    is in the beliefs array and not in the belief set. If so, remove it from the belief set and notify there has
    been any alteration through the boolean value in return.
*/
bool BeliefManager::removedPredicateBeliefs(const vector<Belief>& beliefs)
{
    bool modified = false;//if anything changes, put it to true

    set<ManagedBelief> pred_in_belief_set_ = BDIFilter::extractMGPredicates(belief_set_);
    set<ManagedBelief> pred_in_pddl_prob = BDIFilter::extractMGPredicates(beliefs);

    if(pred_in_belief_set_.size() > pred_in_pddl_prob.size())
    {
        //some predicate has to be removed from the belief set, since it has already been removed from the pddl problem
        for(ManagedBelief mb : pred_in_belief_set_)
            if(pred_in_pddl_prob.count(mb) == 0)
            {
                modified = true;
                delBelief(mb);//mb shall be removed from belief_set_ since it's not present in the pddl_problem
            }
    }
    return modified;
}

/*  
    Someone has publish a new belief in the respective topic
*/
void BeliefManager::addBeliefTopicCallBack(const Belief::SharedPtr msg)
{
    ManagedBelief mb = ManagedBelief{*msg};
    if(psys2_domain_expert_active_ && psys2_problem_expert_active_)
        addBeliefSyncPDDL(mb);
}

/*
    Add Belief in the belief set, just after having appropriately sync the pddl_problem to add it there too
*/
void BeliefManager::addBeliefSyncPDDL(const ManagedBelief& mb)
{   
    bool alreadyThere = true;//belief already in belief set (check later)
    mtx_sync.lock();
        if(belief_set_.count(mb)==0)
        {
            alreadyThere = false;//belief NOT already in belief set (check later)

            if(mb.pddlType() == Belief().INSTANCE_TYPE)
            {   
                //try to add new instance; if fails (word conflicts, wrong/missing type), no biggie!
                Instance ins = BDIPDDLConverter::buildInstance(mb);
                if(problem_expert_->addInstance(ins))
                    addBelief(mb);
            } 

            if(mb.pddlType() == Belief().PREDICATE_TYPE)
            {   
                //try to add new predicate; if fails, try to check and add missing instances
                Predicate p_add = BDIPDDLConverter::buildPredicate(mb);
                if(problem_expert_->addPredicate(p_add) || 
                    tryAddMissingInstances(mb) && problem_expert_->addPredicate(p_add))
                    addBelief(mb);
            } 
            
            if(mb.pddlType() == Belief().FUNCTION_TYPE)
            {   
                //try to add new function; if fails, try to check and add missing instances
                Function f_add =  BDIPDDLConverter::buildFunction(mb);
                if(problem_expert_->addFunction(f_add) || 
                    tryAddMissingInstances(mb) && problem_expert_->addFunction(f_add))
                    addBelief(mb);
            }

        }
        else if(mb.pddlType() == Belief().FUNCTION_TYPE && mb.getValue() != (*(belief_set_.find(mb))).getValue())
        {
            //function present in the belief set with diff. value
            Function f_upd = BDIPDDLConverter::buildFunction(mb);
            if(problem_expert_->updateFunction(f_upd))//instances have to be already present
                modifyBelief(mb);
        }
    mtx_sync.unlock();
    
    if(!alreadyThere && belief_set_.count(mb) > 0)//modification to belief set
        publishBeliefSet();
}

/*
    Create array of boolean flags denoting missing instances' positions
    wrt. parameters in the passed ManagedBelief argument
*/
vector<bool> BeliefManager::computeMissingInstancesPos(const ManagedBelief& mb)
{
    vector<bool> missing_pos = vector<bool>();
    vector<Instance> instances = problem_expert_->getInstances();
    for(ManagedParam mb_par : mb.getParams())
    {    
        bool found = false;

        for(Instance ins : instances)
        {
            if(mb_par.name == ins.name)
            {
                found = true;//instance already defined
                break;
            }
        }

        missing_pos.push_back(!found);//flag denote missing instance
    }
    return missing_pos;
}

/*
    Try adding missing instances (if any)
*/
bool BeliefManager::tryAddMissingInstances(const ManagedBelief& mb)
{   
    vector<bool> missing_pos = computeMissingInstancesPos(mb);
    
    if(this->get_parameter(PARAM_DEBUG).as_bool())
    {
        string missing_pos_string = "";
        for(bool mp : missing_pos)
            missing_pos_string += std::to_string((int)mp) + ", ";
        RCLCPP_INFO(this->get_logger(), "Missing: " + missing_pos_string);
    }
    
    
    if(mb.pddlType() == Belief().PREDICATE_TYPE)
    {   
        try {
            //retrieve from domain expert definition information about this predicate
            Predicate pred = domain_expert_->getPredicate(mb.getName()).value();
            for(int i = 0; i<pred.parameters.size(); i++)
            {
                if(missing_pos[i])//missing instance
                {   
                    ManagedBelief mb_ins = ManagedBelief::buildMBInstance(mb.getParams()[i].name, pred.parameters[i].type);
                    
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Trying to add instance: " + mb_ins.getName() + " - " + mb_ins.type());
                    
                    if(problem_expert_->addInstance(BDIPDDLConverter::buildInstance(mb_ins)))//add instance (type found from domain expert)
                        addBelief(mb_ins);
                    else
                        return false;//add instance failed
                }
            }

        } catch(const std::bad_optional_access& e) {}
    } 
    
    if(mb.pddlType() == Belief().FUNCTION_TYPE)
    {   
        try {
            //retrieve from domain expert definition information about this function
            Function function = domain_expert_->getFunction(mb.getName()).value();
            
            for(int i = 0; i<function.parameters.size(); i++)
            {
                if(missing_pos[i])//missing instance
                {
                        ManagedBelief mb_ins = ManagedBelief::buildMBInstance(mb.getParams()[i].name, function.parameters[i].type);
                    
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Trying to add instance: " + mb_ins.getName() + " - " + mb_ins.type());
                    
                    if(problem_expert_->addInstance(BDIPDDLConverter::buildInstance(mb_ins)))//add instance (type found from domain expert)
                        addBelief(mb_ins);
                    else
                        return false;//add instance failed
                }

            }

        } catch(const std::bad_optional_access& e) {}
    }

    return true;
}

/*  
    Someone has publish a belief to be removed in the respective topic
*/
void BeliefManager::delBeliefTopicCallBack(const Belief::SharedPtr msg)
{
    ManagedBelief mb = ManagedBelief{*msg};
    // if(this->get_parameter(PARAM_DEBUG).as_bool())
    //     RCLCPP_INFO(this->get_logger(), "del_belief callback for " + mb.pddlTypeString() + ": " + mb.getName() + " " 
    //             + mb.getParamsJoined() +  " (value = " + std::to_string(mb.getValue()) +")");
    
    if(psys2_domain_expert_active_ && psys2_problem_expert_active_)
        delBeliefSyncPDDL(mb);
}


/*
    Remove Belief from the belief set, just after having appropriately sync the pddl_problem to remove it from there too
*/
void BeliefManager::delBeliefSyncPDDL(const ManagedBelief& mb)
{
    bool done = false;
    mtx_sync.lock();
        if(belief_set_.count(mb)==1)
        {
            if(mb.pddlType() == Belief().INSTANCE_TYPE)
            {
                //relative predicates/functions will be automatically removed in the pddl_problem, 
                // hence in the consequent update notification removed in the belief_set as well
                Instance ins = BDIPDDLConverter::buildInstance(mb);
                done = !problem_expert_->getInstance(ins.name).has_value() || problem_expert_->removeInstance(ins);
            }

            if(mb.pddlType() == Belief().PREDICATE_TYPE)
            {
                Predicate pred = BDIPDDLConverter::buildPredicate(mb);
                done = !problem_expert_->existPredicate(pred) || problem_expert_->removePredicate(pred);
            }

            if(mb.pddlType() == Belief().FUNCTION_TYPE)
            {
                Function fun = BDIPDDLConverter::buildFunction(mb);
                done = !problem_expert_->existFunction(fun) || problem_expert_->removeFunction(fun);
            }
            
            if(done)
                belief_set_.erase(mb);
        }
    mtx_sync.unlock();

    if(done)//modification has happened, publish new belief set
        publishBeliefSet();
}

/*
    add belief into belief set
*/
void BeliefManager::addBelief(const ManagedBelief& mb)
{
    belief_set_.insert(mb);
    if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_INFO(this->get_logger(), "Added belief ("+mb.pddlTypeString()+"): " + 
            mb.getName() + " " + (mb.pddlType() == Belief().INSTANCE_TYPE? mb.type() : mb.getParamsJoined()) + 
                ((mb.pddlType() == Belief().FUNCTION_TYPE)?
                " (value = " + std::to_string(mb.getValue()) +")" : "")
            );
                    
}

/*
    remove and add belief into belief set 
    (i.e. cover the case of same function with diff. values)
*/
void BeliefManager::modifyBelief(const ManagedBelief& mb)
{
    if(belief_set_.count(mb) == 1){
        belief_set_.erase(mb);
        belief_set_.insert(mb);
        if(this->get_parameter(PARAM_DEBUG).as_bool())
            RCLCPP_INFO(this->get_logger(), "Modified belief ("+mb.pddlTypeString()+"): " + 
                mb.getName() + " " + (mb.pddlType() == Belief().INSTANCE_TYPE? mb.type() : mb.getParamsJoined()) + 
                ((mb.pddlType() == Belief().FUNCTION_TYPE)?
                " (value = " + std::to_string(mb.getValue()) +")" : "")
            );
    }
}

/*
    delete belief from belief set
*/
void BeliefManager::delBelief(const ManagedBelief& mb)
{
    belief_set_.erase(mb);
    if(this->get_parameter(PARAM_DEBUG).as_bool())
        RCLCPP_INFO(this->get_logger(), "Removed belief ("+mb.pddlTypeString()+"): " + 
            mb.getName() + " " + (mb.pddlType() == Belief().INSTANCE_TYPE? mb.type() : mb.getParamsJoined()) + 
            ((mb.pddlType() == Belief().FUNCTION_TYPE)?
                " (value = " + std::to_string(mb.getValue()) +")" : "")
            );
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BeliefManager>();
  bool psys2_booted = node->wait_psys2_boot(std::chrono::seconds(8));//Wait max 8 seconds for plansys2 to boot
  
  if(psys2_booted)
  {
    node->init();
    rclcpp::spin(node);
  }
  else
  {
    std::cerr << "PlanSys2 failed to boot: node will not spin and process will terminate" << std::endl;
  }
  
  rclcpp::shutdown();

  return 0;
}
