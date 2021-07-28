#include <cstdlib>
#include <optional>
#include <ctime>
#include <memory>
#include <mutex>
#include <vector>
#include <set>   
#include <map>   
#include <thread>    

#include <yaml-cpp/exceptions.h>

#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_interfaces/msg/plan_sys2_state.hpp"
#include "ros2_bdi_interfaces/msg/bdi_action_execution_info.hpp"
#include "ros2_bdi_interfaces/msg/bdi_plan_execution_info.hpp"
#include "ros2_bdi_interfaces/srv/bdi_plan_execution.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"
#include "ros2_bdi_utils/PDDLBDIConverter.hpp"
#include "ros2_bdi_utils/BDIYAMLParser.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/ManagedPlan.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_COMM_ERRORS 16
#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"
#define PARAM_MAX_TRIES_DISCARD "tries_desire_discard"

using std::string;
using std::vector;
using std::set;
using std::map;
using std::thread;
using std::mutex;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::optional;

using plansys2::DomainExpertClient;
using plansys2::ProblemExpertClient;
using plansys2::PlannerClient;
using plansys2::Goal;
using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::DesireSet;
using ros2_bdi_interfaces::msg::PlanSys2State;
using ros2_bdi_interfaces::msg::BDIActionExecutionInfo;
using ros2_bdi_interfaces::msg::BDIPlanExecutionInfo;
using ros2_bdi_interfaces::srv::BDIPlanExecution;

typedef enum {STARTING, SCHEDULING, PAUSE} StateType;                

class Scheduler : public rclcpp::Node
{
public:
  Scheduler()
  : rclcpp::Node("scheduler"), state_(STARTING)
  {
    psys2_comm_errors_ = 0;
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);
    this->declare_parameter(PARAM_MAX_TRIES_DISCARD, 4);

  }

  /*
    Init to call at the start, after construction method, to get the node actually started
    initialing planner client instance, 
    retrieving agent_id_ (thus namespace)
    defining work timer,
    belief set subscriber callback,
    desire set publisher,
    add/del desire subscribers callback
  */
  void init()
  { 
    //agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    // initializing domain expert
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    // initializing problem expert
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();

    // initializing planner client
    planner_client_ = std::make_shared<plansys2::PlannerClient>();

    //Init desire set
    desire_set_ = set<ManagedDesire>();

    //Desire set publisher
    desire_set_publisher_ = this->create_publisher<DesireSet>("desire_set", 10);
    
    rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
    qos_keep_all.keep_all();

    //Check for plansys2 active state flags init to false
    psys2_planner_active_ = false;
    psys2_domain_expert_active_ = false;
    psys2_problem_expert_active_ = false;
    //plansys2 nodes status subscriber (receive notification from plansys2_monitor node)
    plansys2_status_subscriber_ = this->create_subscription<PlanSys2State>(
                "plansys2_state", qos_keep_all,
                bind(&Scheduler::callbackPsys2State, this, _1));

    //Desire to be added notification
    add_desire_subscriber_ = this->create_subscription<Desire>(
                "add_desire", qos_keep_all,
                bind(&Scheduler::addDesireTopicCallBack, this, _1));

    //Desire to be removed notification
    del_desire_subscriber_ = this->create_subscription<Desire>(
                "del_desire", qos_keep_all,
                bind(&Scheduler::delDesireTopicCallBack, this, _1));

    //belief_set_subscriber_ 
    belief_set_subscriber_ = this->create_subscription<BeliefSet>(
                "belief_set", qos_keep_all,
                bind(&Scheduler::updatedBeliefSet, this, _1));

    plan_exec_info_subscriber_ = this->create_subscription<BDIPlanExecutionInfo>(
        "plan_execution_info", 10,
         bind(&Scheduler::updatePlanExecution, this, _1)
    );

    //loop to be called regularly to perform work (publish belief_set_, sync with plansys2 problem_expert node...)
    do_work_timer_ = this->create_wall_timer(
        milliseconds(500),
        bind(&Scheduler::step, this));

    RCLCPP_INFO(this->get_logger(), "Scheduler node initialized");
  }
  
  /*
    Main loop of work called regularly through a wall timer
  */
  void step()
  {
    //if psys2 appears crashed, crash too
    if(psys2_comm_errors_ > MAX_COMM_ERRORS)
        rclcpp::shutdown();

    switch (state_) {
        
        case STARTING:
        {
            if(psys2_planner_active_ && psys2_domain_expert_active_ && psys2_problem_expert_active_){
                psys2_comm_errors_ = 0;
                tryInitDesireSet();
                setState(SCHEDULING);
            }else{
                
                if(!psys2_planner_active_)
                    RCLCPP_ERROR(this->get_logger(), "PlanSys2 Planner still not active");

                if(!psys2_domain_expert_active_)
                    RCLCPP_ERROR(this->get_logger(), "PlanSys2 Domain Expert still not active");

                if(!psys2_problem_expert_active_)
                    RCLCPP_ERROR(this->get_logger(), "PlanSys2 Problem Expert still not active");


                psys2_comm_errors_++;
            }

            break;
        }

        case SCHEDULING:
        {   
            publishDesireSet();
            bool noPlan = noPlanSelected();
          
            if(noPlan)
            {
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Reschedule to select new plan to be executed");

                reschedule();

            }else{
                //already selected a plan currently in exec
            }
        }

        case PAUSE:
        {   
            //RCLCPP_INFO(this->get_logger(), "Not the moment to ask for new cleaning tasks yet");    
            break;
        }
        default:
            break;
    }
  }

private:
    /*  
        Change internal state of the node
    */
    void setState(StateType state)
    {
        state_ = state;
    }

    /*
       Publish the current desire set of the agent in agent_id_/desire_set topic
    */
    void publishDesireSet()
    {
        DesireSet dset_msg = BDIFilter::extractDesireSetMsg(desire_set_);
        desire_set_publisher_->publish(dset_msg);
    }

    /*
       Received notification about PlanSys2 nodes state by plansys2 monitor node
    */
    void callbackPsys2State(const PlanSys2State::SharedPtr msg)
    {
        psys2_problem_expert_active_ = msg->problem_expert_active;
        psys2_domain_expert_active_ = msg->domain_expert_active;
        psys2_planner_active_ = msg->planner_active;
    }

    /*
        Expect to find yaml file to init the desire set in "/tmp/{agent_id}/init_dset.yaml"
    */
    void tryInitDesireSet()
    {
        string init_dset_filepath = "/tmp/"+this->get_parameter("agent_id").as_string()+"/init_dset.yaml";
        try{
            vector<ManagedDesire> init_mgdesires = BDIYAMLParser::extractMGDesires(init_dset_filepath);
            for(ManagedDesire initMGDesire : init_mgdesires)
                if(initMGDesire.getValue().size() > 0)
                        addDesire(initMGDesire);
            if(this->get_parameter(PARAM_DEBUG).as_bool())
                RCLCPP_INFO(this->get_logger(), "Desire set initialization performed through " + init_dset_filepath);
        
        }catch(const YAML::BadFile& bfile){
            RCLCPP_ERROR(this->get_logger(), "Bad File: Desire set initialization failed because init. file " + init_dset_filepath + " hasn't been found");
        }catch(const YAML::ParserException& bpars){
            RCLCPP_ERROR(this->get_logger(), "YAML Parser Exception: Desire set initialization failed because init. file " + init_dset_filepath + " doesn't present a valid YAML format");
        }catch(const YAML::BadConversion& bconvfile){
            RCLCPP_ERROR(this->get_logger(), "Bad Conversion: Desire set initialization failed because init. file " + init_dset_filepath + " doesn't present a valid desire array");
        }catch(const YAML::InvalidNode& invalid_node){
            RCLCPP_ERROR(this->get_logger(), "Invalid Node: Desire set initialization failed because init. file " + init_dset_filepath + " doesn't present a valid desire array");
        }   
    }

    /*
        Check with the problem_expert to understand if this is a valid goal
        (i.e. valid predicates and valid instances defined within them)
    */
    bool validGoal(const ManagedDesire& md)
    {
        for(ManagedBelief mb : md.getValue())
        {
            optional<Predicate> opt_pred = domain_expert_->getPredicate(mb.getName());
            if(!opt_pred.has_value())//incorrect predicate name
                return false;

            Predicate pred = opt_pred.value();
            if(pred.parameters.size() != mb.getParams().size())
            {
                RCLCPP_ERROR(this->get_logger(), "pred.parameters.size = %d, mb.getParams().size() = %d", 
                    pred.parameters.size(), mb.getParams().size());
                return false;
            }
            
            for(string ins_name : mb.getParams())
                if(!problem_expert_->getInstance(ins_name).has_value())//found a not valid instance in one of the goal predicates
                    return false;
            
        }
        
        return true;
    }

    /*
        Compute plan from managed desire, setting its belief array representing the desirable state to reach
        as the goal of the PDDL problem 
    */
    optional<Plan> computePlan(const ManagedDesire& md)
    {   
        //set desire as goal of the pddl_problem
        if(!problem_expert_->setGoal(Goal{PDDLBDIConverter::desireToGoal(md.toDesire())})){
            psys2_comm_errors_++;//plansys2 comm. errors
            return std::nullopt;
        }

        string pddl_domain = domain_expert_->getDomain();//get domain string
        string pddl_problem = problem_expert_->getProblem();//get problem string
        return planner_client_->getPlan(pddl_domain, pddl_problem);//compute plan (n.b. goal unfeasible -> plan not computed)
    }

    /*
        Check if there is a current valid plan selected
    */
    bool noPlanSelected()
    {
        return current_plan_.getDesire().getPriority() == 0.0f || current_plan_.getBody().size() == 0;
    }

    /*
        Select plan execution based on precondition, deadline
    */
    void reschedule()
    {   
        // priority of selected plan
        float highestPriority = 0.0f;
        // deadline of selected plan
        float selectedDeadline = -1.0f;//  init to negative value
        
        ManagedPlan selectedPlan;
        
        vector<ManagedDesire> discarded_desires;

        for(ManagedDesire md : desire_set_)
        {
            //select just desires of higher or equal priority with respect to the one currently selected
            if(md.getPriority() >= highestPriority){
                optional<Plan> opt_p = computePlan(md);
                if(opt_p.has_value())
                {
                    ManagedPlan mp = ManagedPlan{md, opt_p.value().items};
                    // does computed deadline for this plan respect desire deadline?
                    if(mp.getPlanDeadline() <= md.getDeadline()) 
                    {
                        // pick it as selected plan iff: no plan selected yet || desire has higher priority than the one selected
                        // or equal priority, but smaller deadline
                        if(selectedDeadline < 0 || md.getPriority() > highestPriority || mp.getPlanDeadline() < selectedDeadline)
                        {    
                            selectedDeadline = mp.getPlanDeadline();
                            highestPriority = md.getPriority();
                            selectedPlan = mp;
                        
                        }else if(md.getPriority() <= highestPriority && this->get_parameter(PARAM_DEBUG).as_bool()){
                            RCLCPP_INFO(this->get_logger(), "There is a plan to fulfill desire \"" + md.getName() + "\", but "+ 
                                "it it's not the desire (among which a plan can be selected) with highest priority right now");

                        }else if(mp.getPlanDeadline() >= selectedDeadline && this->get_parameter(PARAM_DEBUG).as_bool()){
                            RCLCPP_INFO(this->get_logger(), "There is a plan to fulfill desire \"" + md.getName() + "\", but "+
                                "it it's not the desire (among which a plan can be selected) with highest priority and earliest deadline right now");
                        }

                    }else if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "There is a plan to fulfill desire \"" + md.getName() + "\", but it does not respect the deadline constraint");

                }
                else if(!validGoal(md)) //check if the problem is the goal not being valid          
                {
                    int invCounter = ++invalid_desire_map_[md.getName()]; //increment invalid counter for this desire
                    int maxTries = this->get_parameter(PARAM_MAX_TRIES_DISCARD).as_int();

                    string desireOperation = (invCounter < maxTries)? "desire will be rescheduled later" : "desire will be deleted from desire set";
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Desire \"" + md.getName() + "\" presents not valid goal: " +  
                            desireOperation + " (invalid counter = %d).", invCounter);
                    
                    if(invCounter >= maxTries)//desire now has to be discarded
                        discarded_desires.push_back(md);// plan to delete desire from desire_set (not doing here because we're cycling on desire)
                }
            }
            
        }

        //removed discarded desires
        for(ManagedDesire md : discarded_desires)
            delDesire(md);

        //check that a proper plan has been selected (with actions and fulfilling a desire in the desire_set_)
        if(selectedPlan.getBody().size() > 0 && desire_set_.count(selectedPlan.getDesire())==1)
        {
            current_plan_ = selectedPlan;
            if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Ready to execute plan for desire \"" + current_plan_.getDesire().getName() + "\"");
                    
            //trigger plan execution
            shared_ptr<thread> thr = 
                std::make_shared<thread>(bind(&Scheduler::triggerPlanExecution, this, BDIPlanExecution::Request().EXECUTE));
            thr->detach();
        }
    }

    /*
        Contact plan executor to trigger the execution of the current_plan_
    */
    void triggerPlanExecution(const int& PLAN_EXEC_REQUEST)
    {
        //check for service to be up
        rclcpp::Client<BDIPlanExecution>::SharedPtr client_ = 
            this->create_client<BDIPlanExecution>("plan_execution");
       
        int err = 0;
        while(!client_->wait_for_service(std::chrono::seconds(1))){
            //service seems not even present, just return false
            RCLCPP_WARN(this->get_logger(), "plan_execution server does not appear to be up");
            if(err==3)
                return;
            err++;
        }

        auto request = std::make_shared<BDIPlanExecution::Request>();
        request->request = PLAN_EXEC_REQUEST;
        request->plan = current_plan_.toPlan();
        auto future = client_->async_send_request(request);

        try{
            auto response = future.get();
            if(PLAN_EXEC_REQUEST == request->EXECUTE && !response->success)
            {
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Triggered new plan execution failed");
                current_plan_ = ManagedPlan{};//notifying you're not executing any plan right now
                reschedule();
            }
            else if(PLAN_EXEC_REQUEST == request->EXECUTE && response->success)
            {
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Triggered new plan execution success");
            }
            else if(PLAN_EXEC_REQUEST == request->ABORT && response->success)
            {
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Aborted plan execution");
                
                current_plan_ = ManagedPlan{};//notifying you're not executing any plan right now
                reschedule();
            }
        }catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Response error in executor/get_state");
        }
    }

    /*
        Received update on current plan execution
    */
    void updatePlanExecution(const BDIPlanExecutionInfo::SharedPtr msg)
    {
        auto planExecInfo = (*msg);
        if(!noPlanSelected() && planExecInfo.target.name == current_plan_.getDesire().getName())//current plan selected in execution update
        {
            if(planExecInfo.status != planExecInfo.RUNNING)//plan not running anymore
            {
                if(planExecInfo.status == planExecInfo.SUCCESSFUL)//plan exec completed successful
                {
                    bool desireAchieved = isDesireSatisfied(planExecInfo.target);
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                    {   
                        string addNote = desireAchieved? "desire achieved will be removed from desire set" : "desire still not achieved! It'll not removed from the desire set yet";
                        RCLCPP_INFO(this->get_logger(), "Plan successfully executed: " + addNote);
                    }

                    if(desireAchieved)
                        delDesire(ManagedDesire{planExecInfo.target});//desire achieved
                }

                else if(planExecInfo.status == planExecInfo.ABORT)// plan exec aborted
                {
                    //  for now mantain the desire
                    // (if not valid anymore, it'll be removed in next reschedulings, otherwise the plan will be commissioned again)
                }

                current_plan_ = ManagedPlan{};//no current plan in execution
                reschedule();// reschedule for new plan execution
            }
        }
    }

    /*
        Given the current knowledge of the belief set, decide if a given desire
        is already fulfilled
    */
    bool isDesireSatisfied(const ManagedDesire& md)
    {
        for(ManagedBelief targetb : md.getValue())
            if(belief_set_.count(targetb) == 0)
                return false;//desire still not achieved
                
        return true;//all target conditions already met       
    }

    /*  Use the updated belief set for deciding if some desires are pointless to pursue given the current 
        beliefs which shows they're already fulfilled
    */
    void checkForSatisfiedDesires()
    {
        for(ManagedDesire md : desire_set_)
        {   
            if(isDesireSatisfied(md))//desire already achieved, remove it
            {
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Desire \"" + md.getName() + "\" will be removed from the desire set since its "+
                        "target appears to be already fulfilled given the current belief set");
                
                delDesire(md);

                if(!noPlanSelected() && current_plan_.getDesire() == md)
                {
                    //abort plan execution since current target desire is already achieved
                    shared_ptr<thread> thr = 
                        std::make_shared<thread>(bind(&Scheduler::triggerPlanExecution, this, BDIPlanExecution::Request().ABORT));
                    thr->detach();
                }
            }
        }
    }

    /*
        The belief set has been updated
    */
    void updatedBeliefSet(const BeliefSet::SharedPtr msg)
    {
        belief_set_ = BDIFilter::extractMGBeliefs(msg->value);
        checkForSatisfiedDesires();
    }

    /*  
        Someone has publish a new desire to be fulfilled in the respective topic
    */
    void addDesireTopicCallBack(const Desire::SharedPtr msg)
    {   
        bool added = addDesire(ManagedDesire{(*msg)});

        if(added)//if any alteration to the desire_set has occured, reschedule
            reschedule();
    }

    /*  
        Someone has publish a desire to be removed from the one to be fulfilled (if present)
        in the respective topic
    */
    void delDesireTopicCallBack(const Desire::SharedPtr msg)
    {
        bool deleted = delDesire(ManagedDesire{(*msg)});

        if(deleted)//if any alteration to the desire_set has occured, reschedule
            reschedule();
    }

    /*
        Add desire to desire_set (if there is not yet)
    */
    bool addDesire(const ManagedDesire& md)
    {
        bool added = false;
        mtx_sync.lock();
            if(invalid_desire_map_.count(md.getName())==0 && desire_set_.count(md)==0)//desire already there (or diff. desire but with same name identifier)
            {
                desire_set_.insert(md);
                invalid_desire_map_.insert(std::pair<string, int>(md.getName(), 0));//to count invalid goal computations and discard after x
                added = true;
            }
        mtx_sync.unlock();
        return added;
    }
    
    /*
        Del desire from desire_set if present (Access through lock!)
    */
    bool delDesire(const ManagedDesire& md)
    {
        bool deleted = false;
        mtx_sync.lock();
            if(desire_set_.count(md)!=0)
            {
                desire_set_.erase(desire_set_.find(md));
                invalid_desire_map_.erase(md.getName());
                deleted = true;
                //RCLCPP_INFO(this->get_logger(), "Desire removed!");//TODO remove when assured there is no bug in deletion
            }
        mtx_sync.unlock();
        return deleted;
    }

    // internal state of the node
    StateType state_;
    
    // agent id that defines the namespace in which the node operates
    string agent_id_;
    // callback to perform main loop of work regularly
    rclcpp::TimerBase::SharedPtr do_work_timer_;

    // counter of communication errors with plansys2
    int psys2_comm_errors_;
    // problem expert instance to call the plansys2 problem expert api
    shared_ptr<ProblemExpertClient> problem_expert_;
    // domain expert instance to call the plansys2 domain expert api
    shared_ptr<DomainExpertClient> domain_expert_;
    // planner expert instance to call the plansys2 planner api
    shared_ptr<PlannerClient> planner_client_;
    
    // flag to denote if the plansys2 planner is up and active
    bool psys2_planner_active_;
    // flag to denote if the plansys2 domain expert is up and active
    bool psys2_domain_expert_active_;
    // flag to denote if the plansys2 problem expert planner is up and active
    bool psys2_problem_expert_active_;
    // plansys2 node status monitor subscription
    rclcpp::Subscription<PlanSys2State>::SharedPtr plansys2_status_subscriber_;

    // belief set of the agent <agent_id_>
    set<ManagedBelief> belief_set_;

    // desire set of the agent <agent_id_>
    set<ManagedDesire> desire_set_;

    // hashmap for invalid desire counters (after x tries desire will be discarded)
    map<string, int> invalid_desire_map_;

    // current_plan_ in execution (could be none if the agent isn't doing anything)
    ManagedPlan current_plan_;

    //mutex for sync when modifying desire_set
    mutex mtx_sync;

    // string representing a pddl domain to supply to the plansys2 planner
    string pddl_domain_;
    // string representing a pddl problem to supply to the plansys2 planner
    string pddl_problem_;

    // desire set publishers/subscribers
    rclcpp::Subscription<Desire>::SharedPtr add_desire_subscriber_;//add desire notify on topic
    rclcpp::Subscription<Desire>::SharedPtr del_desire_subscriber_;//del desire notify on topic
    rclcpp::Publisher<DesireSet>::SharedPtr desire_set_publisher_;//desire set publisher

    // belief set subscriber
    rclcpp::Subscription<BeliefSet>::SharedPtr belief_set_subscriber_;//belief set sub.

    // plan executioninfo subscriber
    rclcpp::Subscription<BDIPlanExecutionInfo>::SharedPtr plan_exec_info_subscriber_;//plan execution info publisher
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Scheduler>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
