#include <cstdlib>
#include <optional>
#include <ctime>
#include <memory>
#include <mutex>
#include <vector>
#include <set>   
#include <thread>    

#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_interfaces/msg/bdi_action_execution_info.hpp"
#include "ros2_bdi_interfaces/msg/bdi_plan_execution_info.hpp"
#include "ros2_bdi_interfaces/srv/bdi_plan_execution.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"
#include "ros2_bdi_utils/PDDLBDIConverter.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/ManagedPlan.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_COMM_ERRORS 16
#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"
#define PARAM_PDDL_DOMAIN "pddl_test_domain" 
#define PARAM_PDDL_PROBLEM "pddl_test_problem" 

using std::string;
using std::vector;
using std::set;
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

    //define empty test PDDL domain and problem (loaded up from launch file)
    this->declare_parameter(PARAM_PDDL_DOMAIN, "(define (domain cleaner-domain) (:requirements :strips))");
    this->declare_parameter(PARAM_PDDL_PROBLEM, "(define (problem cleaner-problem) (:domain cleaner-domain))");
    planner_expert_up_ = false;
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

    //Desire to be added notification
    add_desire_subscriber_ = this->create_subscription<Desire>(
                "add_desire", 10,
                bind(&Scheduler::addDesireTopicCallBack, this, _1));

    //Desire to be removed notification
    del_desire_subscriber_ = this->create_subscription<Desire>(
                "del_desire", 10,
                bind(&Scheduler::delDesireTopicCallBack, this, _1));

    //belief_set_subscriber_ 
    belief_set_subscriber_ = this->create_subscription<BeliefSet>(
                "belief_set", 10,
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
            if(isPlannerActive()){
                planner_expert_up_ = true;
                psys2_comm_errors_ = 0;
                setState(SCHEDULING);
            }else{
                planner_expert_up_ = false;
                RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with PlanSys2 planner");
                psys2_comm_errors_++;
            }

            break;
        }

        case SCHEDULING:
        {   
            publishDesireSet();
            reschedule();
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
        Check if the plansys2 planner node is up by checking if it returns the correct plan if
        queried with a test domain and problem
    */
    bool isPlannerActive()
    {   
        bool isUp = false;
        pddl_domain_ = readTestPDDLDomain();
        //std::cout << pddl_domain_ << std::endl << std::endl << std::endl;
        pddl_problem_ = readTestPDDLProblem();
        //std::cout << pddl_problem_ << std::endl << std::endl << std::endl;
        optional<Plan> plan = planner_client_->getPlan(pddl_domain_, pddl_problem_);
        /*if(plan.has_value())
            RCLCPP_INFO(this->get_logger(), "Found plan: start with " + plan.value().items[0].action);*/
        return plan.has_value();
    }

    /*
        Check with the problem_expert to understand if this is a valid goal
        (i.e. valid predicates and valid instances defined within them)
    */
    bool validGoal(const ManagedDesire& md)
    {
        for(ManagedBelief mb : md.value_)
        {
            if(!domain_expert_->getPredicate(mb.name_).has_value())//incorrect predicate name
                return false;

            for(string ins_name : mb.params_)
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
        return current_plan_.desire_.priority_ == 0.0f || current_plan_.body_.size() == 0;
    }

    /*
        Select plan execution based on precondition, deadline
    */
    void reschedule()
    {   
        if(noPlanSelected())
        {
            if(this->get_parameter(PARAM_DEBUG).as_bool())
                RCLCPP_INFO(this->get_logger(), "Reschedule to select new plan to be executed");

            // priority of selected plan
            float highestPriority = 0.0f;
            // deadline of selected plan
            float selectedDeadline = -1.0f;//  init to negative value
            
            ManagedPlan selectedPlan;
            
            vector<ManagedDesire> discarded_desires;

            for(ManagedDesire md : desire_set_)
            {
                //select just desires of higher or equal priority with respect to the one currently selected
                if(md.priority_ >= highestPriority){
                    optional<Plan> opt_p = computePlan(md);
                    if(opt_p.has_value())
                    {
                        ManagedPlan mp = ManagedPlan{md, opt_p.value().items};
                        // does computed deadline for this plan respect desire deadline?
                        if(mp.plan_deadline_ <= md.deadline_) 
                        {
                            // pick it as selected plan iff: no plan selected yet || desire has higher priority than the one selected
                            // or equal priority, but smaller deadline
                            if(selectedDeadline < 0 || md.priority_ > highestPriority || mp.plan_deadline_ < selectedDeadline)
                            {    
                                selectedDeadline = mp.plan_deadline_;
                                highestPriority = md.priority_;
                                selectedPlan = mp;
                            }
                        }
                    }
                    else if(!validGoal(md)) //check if the problem is the goal not being valid          
                    {
                        if(this->get_parameter(PARAM_DEBUG).as_bool())
                            RCLCPP_INFO(this->get_logger(), "Desire \"" + md.name_ + "\" presents not valid goal: desire will be removed from desire_set");
                        discarded_desires.push_back(md);// plan to delete desire from desire_set (not doing here because we're cycling on desire)
                    }
                }
                
            }

            //removed discarded desires
            for(ManagedDesire md : discarded_desires)
                delDesire(md);

            //check that a proper plan has been selected (with actions and fulfilling a desire in the desire_set_)
            if(selectedPlan.body_.size() > 0 && desire_set_.count(selectedPlan.desire_)==1)
            {
                current_plan_ = selectedPlan;
                //trigger plan execution
                shared_ptr<thread> thr = 
                    std::make_shared<thread>(bind(&Scheduler::triggerPlanExecution, this));
                thr->detach();
            }
        }
    }

    /*
        Contact plan executor to trigger the execution of the current_plan_
    */
    void triggerPlanExecution()
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
        request->request = request->EXECUTE;
        request->plan = current_plan_.toPlan();
        auto future = client_->async_send_request(request);

        try{
            auto response = future.get();
            if(!response->success)//notifying you're not executing any plan right now
            {
                current_plan_ = ManagedPlan{};//no current_plan_
            }else{
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Triggered new plan execution");
            }
        }catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Response error in executor/get_state");
        }
    }

    /*
        The belief set has been updated
    */
    void updatePlanExecution(const BDIPlanExecutionInfo::SharedPtr msg)
    {
        auto planExecInfo = (*msg);
        if(planExecInfo.status != planExecInfo.RUNNING)//plan not running anymore
        {
            if(planExecInfo.status == planExecInfo.SUCCESSFUL)//plan exec completed successful
            {
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Plan successfully executed: desire achieved! Remove it from desire set");
                delDesire(ManagedDesire{planExecInfo.target});//desire achieved
            }

            else if(planExecInfo.status == planExecInfo.ABORT)// plan exec aborted
            {
                //  for now mantain the desire
                // (if not valid anymore, it'll be removed in next rescheduling, otherwise the plan will be commissioned again)
            }

            current_plan_ = ManagedPlan{};//no current plan in execution
            reschedule();// reschedule for new plan execution
        }
    }

    /*
        The belief set has been updated
    */
    void updatedBeliefSet(const BeliefSet::SharedPtr msg)
    {
        belief_set_ = BDIFilter::extractMGBeliefs(msg->value);
        // TOCHECK has anything to be called after?
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
            if(desire_set_.count(md)==0)
            {
                desire_set_.insert(md);
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
                deleted = true;
            }
        mtx_sync.unlock();
        return deleted;
    }

    string readTestPDDLDomain()
    {
        return this->get_parameter(PARAM_PDDL_DOMAIN).as_string();
    }

    string readTestPDDLProblem()
    {
        return this->get_parameter(PARAM_PDDL_PROBLEM).as_string();
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
    // flag to denote if the problem expert node seems to correctly answer 
    bool planner_expert_up_;

    // belief set of the agent <agent_id_>
    set<ManagedBelief> belief_set_;

    // desire set of the agent <agent_id_>
    set<ManagedDesire> desire_set_;

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
