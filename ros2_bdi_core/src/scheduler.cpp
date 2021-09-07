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
#include "ros2_bdi_interfaces/msg/condition.hpp"
#include "ros2_bdi_interfaces/msg/conditions_conjunction.hpp"
#include "ros2_bdi_interfaces/msg/conditions_dnf.hpp"
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
#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedConditionsConjunction.hpp"
#include "ros2_bdi_utils/ManagedConditionsDNF.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_COMM_ERRORS 16
#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"
#define PARAM_MAX_TRIES_COMP_PLAN "compute_plan_tries"
#define PARAM_MAX_TRIES_EXEC_PLAN "exec_plan_tries"
#define PARAM_RESCHEDULE_POLICY "reschedule_policy"
#define VAL_RESCHEDULE_POLICY_NO_IF_EXEC "NO_IF_EXEC"
#define VAL_RESCHEDULE_POLICY_IF_EXEC "IF_EXEC"
#define PARAM_AUTOSUBMIT_PREC "autosubmit_precond"
#define PARAM_AUTOSUBMIT_CONTEXT "autosubmit_context"

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
using ros2_bdi_interfaces::msg::Condition;
using ros2_bdi_interfaces::msg::ConditionsConjunction;
using ros2_bdi_interfaces::msg::ConditionsDNF;
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
    this->declare_parameter(PARAM_MAX_TRIES_COMP_PLAN, 8);
    this->declare_parameter(PARAM_MAX_TRIES_EXEC_PLAN, 8);
    this->declare_parameter(PARAM_RESCHEDULE_POLICY, VAL_RESCHEDULE_POLICY_NO_IF_EXEC);
    this->declare_parameter(PARAM_AUTOSUBMIT_PREC, false);
    this->declare_parameter(PARAM_AUTOSUBMIT_CONTEXT, false);

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
    // all psys2 up -> no psys2 comm. errors
    if(psys2_planner_active_ && psys2_domain_expert_active_ && psys2_problem_expert_active_ )
        psys2_comm_errors_ = 0;

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
            string reschedulePolicy = this->get_parameter(PARAM_RESCHEDULE_POLICY).as_string();
            bool noPlan = noPlanSelected();

            /*
                Either the reschedule policy is no if a plan is executing AND there is no plan currently in exec
                or the reschedule policy allows rescheduling while plan is in exec
            */
            if(reschedulePolicy == VAL_RESCHEDULE_POLICY_NO_IF_EXEC && noPlan 
                || reschedulePolicy != VAL_RESCHEDULE_POLICY_NO_IF_EXEC)
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

        TODO change name valida sintatticamente
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
        return current_plan_.getDesire().getPriority() == 0.0f && current_plan_.getBody().size() == 0;
    }

    /*
        Select plan execution based on precondition, deadline
    */
    void reschedule()
    {   
        string reschedulePolicy = this->get_parameter(PARAM_RESCHEDULE_POLICY).as_string();
        bool noPlan = noPlanSelected();
        if(reschedulePolicy == VAL_RESCHEDULE_POLICY_NO_IF_EXEC && !noPlan)//rescheduling not ammitted
            return;
        
        //rescheduling possible, but plan currently in exec (substitute just for plan with higher priority)
        bool planinExec = reschedulePolicy != VAL_RESCHEDULE_POLICY_NO_IF_EXEC && !noPlan;

        // priority of selected plan
        float highestPriority = -1.0f;
        // deadline of selected plan
        float selectedDeadline = -1.0f;//  init to negative value
        
        ManagedPlan selectedPlan;
        
        vector<ManagedDesire> discarded_desires;
        
        mtx_iter_dset_.lock();//to sync between iteration in checkForSatisfiedDesires() && reschedule()

        set<ManagedDesire> skip_desires;

        for(ManagedDesire md : desire_set_)
        {
            if(skip_desires.count(md) == 1)
                continue;

            //plan in exec has higher priority than this one, skip this desire
            if(planinExec && current_plan_.getDesire().getPriority() > md.getPriority())
                continue;
            
            bool computedPlan = false;//flag to mark plan for desire as computable
            bool invalidDesire = false;//flag to mark invalid desire
            
            // select just desires with satisyfing precondition and 
            // with higher or equal priority with respect to the one currently selected
            bool explicitPreconditionSatisfied = md.getPrecondition().isSatisfied(belief_set_);
            if(explicitPreconditionSatisfied && md.getPriority() >= highestPriority){
                optional<Plan> opt_p = computePlan(md);
                if(opt_p.has_value())
                {
                    computedPlan = true;

                    ManagedPlan mp = ManagedPlan{md, opt_p.value().items, md.getPrecondition(), md.getContext()};
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
                    invalidDesire = true;
                }   
                else 
                {
                    if(!this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Desire \"" + md.getName() + "\" presents a valid goal, but planner cannot compute any plan for it at the moment");
                }
            }
            else if(!explicitPreconditionSatisfied && this->get_parameter(PARAM_AUTOSUBMIT_PREC).as_bool())
            {
                int pushed = 0;
                if(validGoal(md))
                {
                    // explicit preconditions are not satisfied... see if it's feasible to compute a plan to reach them
                    // (just if not already done... that's why you look into the invalid map)
                    // if it is the case submit desire to itself with higher priority than the one just considered 
                    string fulfillPreconditionDesireName = md.getName() + "_fulfill_precondition";

                    //put slightly lower priority because these would be desires for satisfy the pre precondition
                    vector<ManagedDesire> fulfillPreconditionDesires = BDIFilter::conditionsToMGDesire(md.getPrecondition(), 
                        fulfillPreconditionDesireName, 
                        std::min(md.getPriority()-0.01f, 1.0f), md.getDeadline());
                    
                    for(ManagedDesire fulfillPreconditionD : fulfillPreconditionDesires)
                    {
                        if(desire_set_.count(fulfillPreconditionD) == 0 && validGoal(fulfillPreconditionD))//fulfill precondition not inserted yet
                        {   
                            if(this->get_parameter(PARAM_DEBUG).as_bool())
                                RCLCPP_INFO(this->get_logger(), "Precondition are not satisfied for desire \"" + md.getName() + "\" but could be satisfied: " +  
                                    +  " auto-submission desire \"" + fulfillPreconditionD.getName() + "\"");
                            fulfillPreconditionD.setParent(md);//set md as its parent desire
                            if(addDesire(fulfillPreconditionD, md, "_preconditions"))
                                pushed++;
                        }
                    }
                }

                if(pushed == 0)
                    invalidDesire = true;
                
            }

            if(invalidDesire || (!computedPlan && explicitPreconditionSatisfied))
            {
                int invCounter = ++computed_plan_desire_map_[md.getName()]; //increment invalid counter for this desire
                int maxTries = this->get_parameter(PARAM_MAX_TRIES_COMP_PLAN).as_int();

                string desireOperation = (invCounter < maxTries)? "desire will be rescheduled later" : "desire will be deleted from desire set";
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Desire \"" + md.getName() + "\" (or its preconditions) presents not valid goal and/or a plan for it is not computable: " +  
                        desireOperation + " (invalid counter = %d).", invCounter);
                
                if(invCounter >= maxTries)//desire now has to be discarded
                {    
                    discarded_desires.push_back(md);// plan to delete desire from desire_set (not doing here because we're cycling on desire)
                    
                    // check if this is trying to satisfy precondition and/or context condition of another desire
                    // and there are no other desires within the same group -> delete that desire too
                    int groupCounter = 0;
                    for(ManagedDesire mdCheck : desire_set_)
                        if(mdCheck.getDesireGroup() == md.getDesireGroup())
                            groupCounter ++;
                    if(md.hasParent() && groupCounter == 0)//invalid desire has remained the last one in the group
                    {
                        discarded_desires.push_back(md);//parent cannot be satisfied either
                        skip_desires.insert(md);//avoid to evaluate it later
                    }
                }
            }
            
        }

        mtx_iter_dset_.unlock();//to sync between iteration in checkForSatisfiedDesires() && reschedule()

        //removed discarded desires
        for(ManagedDesire md : discarded_desires)
            delDesire(md);

        if(selectedPlan.getBody().size() > 0)
            tryTriggerPlanExecution(selectedPlan);
    }


    /*
        If selected plan fit the minimal requirements for a plan (i.e. not empty body and a desire which is in the desire_set)
        try triggering its execution by srv request to PlanDirector (/{agent}/plan_execution)
    */
    void tryTriggerPlanExecution(const ManagedPlan& selectedPlan)
    {      
        string reschedulePolicy = this->get_parameter(PARAM_RESCHEDULE_POLICY).as_string();
        bool noPlan = noPlanSelected();
        //rescheduling not ammitted -> a plan already executing and policy not admit any switch with higher priority plans
        if(reschedulePolicy == VAL_RESCHEDULE_POLICY_NO_IF_EXEC && !noPlan)
            return;

        //rescheduling possible, but plan currently in exec (substitute just for plan with higher priority)
        bool planinExec = reschedulePolicy != VAL_RESCHEDULE_POLICY_NO_IF_EXEC && !noPlan;

        if(planinExec)
        {
            //before triggering new plan, abort the one currently in exec
            if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Ready to abort plan for desire \"" + current_plan_.getDesire().getName() + "\"" + 
                                " in order to trigger plan execution for desire \"" + current_plan_.getDesire().getName() + "\"");
                
            //trigger plan execution
            shared_ptr<thread> thr = 
                std::make_shared<thread>(bind(&Scheduler::triggerPlanExecution, this, 
                BDIPlanExecution::Request().ABORT, current_plan_));
            thr->detach();

            //mutex to handle better ABORT old plan -> EXECUTE new plan sequence
            while(mtx_abort_trigger_.try_lock())//when you fail it means that aborting thread has started
                mtx_abort_trigger_.unlock();
            mtx_abort_trigger_.lock();//wait aborting thread to be finished
            mtx_abort_trigger_.unlock();
        }
        
        //desire still in desire set
        bool desireInDesireSet = desire_set_.count(selectedPlan.getDesire())==1;
        
        //check that a proper plan has been selected (with actions and fulfilling a desire in the desire_set_)
        if(selectedPlan.getBody().size() <= 0 || !desireInDesireSet)
            return;
            
        // selectedPlan can now be set as currently executing plan
        current_plan_ = selectedPlan;
        if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Ready to execute plan for desire \"" + current_plan_.getDesire().getName() + "\"");
                
        //trigger plan execution
        shared_ptr<thread> thr = 
            std::make_shared<thread>(bind(&Scheduler::triggerPlanExecution, this, 
            BDIPlanExecution::Request().EXECUTE, current_plan_));
        thr->detach();

    }

    /*
        Contact plan executor to trigger the execution of the current_plan_
    */
    void triggerPlanExecution(const int& PLAN_EXEC_REQUEST, const ManagedPlan& mgPlan)
    {
        mtx_abort_trigger_.lock();

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
        request->plan = mgPlan.toPlan();
        auto future = client_->async_send_request(request);

        try{
            auto response = future.get();
            if(PLAN_EXEC_REQUEST == request->EXECUTE && !response->success)
            {
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Triggered new plan execution failed");
                
                if(mgPlan == current_plan_)//aborted current plan execution
                    current_plan_ = ManagedPlan{};//notifying you're not executing any plan right now
                    //next reschedule(); will select a new plan if computable for a desire in desire set
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
                
                if(mgPlan == current_plan_)//aborted current plan execution
                    current_plan_ = ManagedPlan{};//notifying you're not executing any plan right now
                    //next reschedule(); will select a new plan if computable for a desire in desire set
            }
        }catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Response error in executor/get_state");
        }

        mtx_abort_trigger_.unlock();
    }

    /*
        Received update on current plan execution
    */
    void updatePlanExecution(const BDIPlanExecutionInfo::SharedPtr msg)
    {
        auto planExecInfo = (*msg);
        ManagedDesire targetDesire = ManagedDesire{planExecInfo.target};

        if(!noPlanSelected() && planExecInfo.target.name == current_plan_.getDesire().getName())//current plan selected in execution update
        {
            current_plan_exec_info_ = planExecInfo;
            string targetDesireName = targetDesire.getName();
            if(planExecInfo.status != planExecInfo.RUNNING)//plan not running anymore
            {
                bool desireAchieved = isDesireSatisfied(targetDesire);
                if(desireAchieved)
                    delDesire(targetDesire, true);//desire achieved -> delete all desires within the same group
                
                if(planExecInfo.status == planExecInfo.SUCCESSFUL)//plan exec completed successful
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                    {   
                        string addNote = desireAchieved? 
                            "desire \"" + targetDesireName + "\" achieved will be removed from desire set" : 
                            "desire \"" + targetDesireName + "\" still not achieved! It'll not removed from the desire set yet";
                        if(!desireAchieved)
                        {
                            std::cout << "\n\n";
                            std::cout << "\ndesire: " << targetDesire << std::endl;
                            for(auto mb : belief_set_)
                                std:: cout << "\n" << mb;
                            std::cout << "\n\n";
                        }
                        RCLCPP_INFO(this->get_logger(), "Plan successfully executed: " + addNote);
                    }
                }

                else if(planExecInfo.status == planExecInfo.ABORT && !desireAchieved)// plan exec aborted and desire not achieved
                {

                    int maxPlanExecAttempts = this->get_parameter(PARAM_MAX_TRIES_EXEC_PLAN).as_int();
                    aborted_plan_desire_map_[targetDesireName]++;
                    
                    RCLCPP_INFO(this->get_logger(), "Plan execution for fulfilling desire \"" + targetDesireName + 
                        "\" has been aborted for the %d time (max attempts: %d)", 
                            aborted_plan_desire_map_[targetDesireName], maxPlanExecAttempts);
                    
                    if(aborted_plan_desire_map_[targetDesireName] >= maxPlanExecAttempts)
                    {
                        if(this->get_parameter(PARAM_DEBUG).as_bool())
                            RCLCPP_INFO(this->get_logger(), "Desire \"" + targetDesireName + "\" will be removed because it doesn't seem feasible to fulfill it: too many plan abortions!");
                        delDesire(targetDesire);
                    
                    }else if(!targetDesire.getContext().isSatisfied(belief_set_) && this->get_parameter(PARAM_AUTOSUBMIT_CONTEXT).as_bool()){
                        // check for context condition failed 
                        // (just if not already done... that's why you look into the invalid map)
                        // plan exec could have failed cause of them: evaluate if they can be reached and submit the desire to yourself
                        string fulfillContextDesireName = targetDesire.getName() + "_fulfill_context";
                        
                        // extract a desire (if possible) for each clause in the context conditions
                        vector<ManagedDesire> fulfillContextDesires = BDIFilter::conditionsToMGDesire(targetDesire.getContext(), 
                            fulfillContextDesireName, 
                            std::min(targetDesire.getPriority()+0.01f, 1.0f), targetDesire.getDeadline());
                        
                        for(ManagedDesire fulfillContextD : fulfillContextDesires)
                        {
                            if(desire_set_.count(fulfillContextD) == 0 && validGoal(fulfillContextD))
                            {
                                if(this->get_parameter(PARAM_DEBUG).as_bool())
                                    RCLCPP_INFO(this->get_logger(), "Context conditions are not satisfied for desire \"" + targetDesire.getName() + "\" but could be satisfied: " +  
                                        +  " auto-submission desire \"" + fulfillContextD.getName() + "\"");
                                fulfillContextD.setParent(targetDesire);//set md as its parent desire
                                addDesire(fulfillContextD, targetDesire, "_context");
                            }
                        }
                        
                    }

                    //  if not reached max exec attempt, for now mantain the desire
                    //  if not valid anymore, it'll be eventually removed in next reschedulings, 
                    //  otherwise the plan will be commissioned again until reaching maxPlanExecAttempts

                        
                }

                current_plan_ = ManagedPlan{};//no current plan in execution
                //next reschedule(); will select a new plan if computable for a desire in desire set
            }
        }
    }

    /*
        Given the current knowledge of the belief set, decide if a given desire
        is already fulfilled
    */
    bool isDesireSatisfied(ManagedDesire& md)
    {
        return md.isFulfilled(belief_set_);
    }

    /*  Use the updated belief set for deciding if some desires are pointless to pursue given the current 
        beliefs which shows they're already fulfilled
    */
    void checkForSatisfiedDesires()
    {
        mtx_iter_dset_.lock();//to sync between iteration in checkForSatisfiedDesires() && reschedule()

        vector<ManagedDesire> satisfiedDesires;
        for(ManagedDesire md : desire_set_)
        {   
            if(isDesireSatisfied(md))//desire already achieved, remove it
            {
                if(!noPlanSelected() && current_plan_.getDesire() == md && 
                    current_plan_exec_info_.status == current_plan_exec_info_.RUNNING && 
                    current_plan_exec_info_.executing.size() > 0 && !executingLastAction())
                {
                    int lastActionIndex = current_plan_exec_info_.actions.size() -1;
                    int executingActionIndex = -1;
                    for(auto exec_action : current_plan_exec_info_.executing)
                        executingActionIndex = std::max((int)exec_action.index, executingActionIndex);

                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                        RCLCPP_INFO(this->get_logger(), "Current plan execution fulfilling desire \"" + md.getName() + 
                            "\" will be aborted since desire is already fulfilled and plan exec. is still far from being completed " +
                            "(executing %d out of %d)", lastActionIndex, executingActionIndex);

                    //abort current_plan_ plan execution since current target desire is already achieved and you're far from completing the plan (missing more than last action)
                    shared_ptr<thread> thr = 
                        std::make_shared<thread>(bind(&Scheduler::triggerPlanExecution, this, 
                        BDIPlanExecution::Request().ABORT, current_plan_));
                    thr->detach();
                }
                else
                {
                    if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Desire \"" + md.getName() + "\" will be removed from the desire set since its "+
                        "target appears to be already fulfilled given the current belief set");
                
                    satisfiedDesires.push_back(md);//delete desire just if not executing one, otherwise will be deleted when aborted feedback comes and desire is satisfied
                }

            }
        }

        mtx_iter_dset_.unlock();//to sync between iteration in checkForSatisfiedDesires() && reschedule()

        for(ManagedDesire md : satisfiedDesires)//delete all satisfied desires from desire set
            delDesire(md, true);//you're deleting satisfied desires -> delete all the ones in the same group
    }

    /*
        wrt the current plan execution...
        return true iff currently executing last action
        return false if otherwise or not executing any plan
    */
    bool executingLastAction()
    {   
        // not exeuting any plan or last update not related to currently triggered plan
        if(noPlanSelected() || current_plan_exec_info_.target.name != current_plan_.getDesire().getName())
            return false;

        bool executingLast = false;
        for(auto currentExecutingAction : current_plan_exec_info_.executing)
            if(currentExecutingAction.index == current_plan_exec_info_.actions.size() - 1)//currentlyExecutingLast action
                executingLast = true;

        return executingLast;
    }

    /*
        The belief set has been updated
    */
    void updatedBeliefSet(const BeliefSet::SharedPtr msg)
    {
        set<ManagedBelief> newBeliefSet = BDIFilter::extractMGBeliefs(msg->value);
        bool bsetModified = false;//is belief set altered from last update?
        
        for(ManagedBelief mb : newBeliefSet)
            if(belief_set_.count(mb) == 0)//check if new belief set has new items
            {
                bsetModified = true;
                break;
            }
        
        if(!bsetModified)
            for(ManagedBelief mb : belief_set_)
                if(newBeliefSet.count(mb) == 0)//check if new belief set has lost a few items
                {
                    bsetModified = true;
                    break;
                }

        if(bsetModified)//if belief set appears different from last update
        {
            belief_set_ = newBeliefSet;//update current mirroring of the belief set
            
            checkForSatisfiedDesires();//check for satisfied desires
            reschedule();//do a rescheduling
        }
        
    }

    /*  
        Someone has publish a new desire to be fulfilled in the respective topic
    */
    void addDesireTopicCallBack(const Desire::SharedPtr msg)
    {   
        bool added = addDesire(ManagedDesire{(*msg)});

        if(added && noPlanSelected())//addition done and no Plan currently selected, reschedule immediately
            reschedule();
    }

    /*  
        Someone has publish a desire to be removed from the one to be fulfilled (if present)
        in the respective topic
    */
    void delDesireTopicCallBack(const Desire::SharedPtr msg)
    {
        bool deleted = delDesire(ManagedDesire{(*msg)});

        if(deleted && ManagedDesire{(*msg)} == current_plan_.getDesire())//deleted desire of current executing plan
        {
            //abort current plan execution
            shared_ptr<thread> thr = 
                std::make_shared<thread>(bind(&Scheduler::triggerPlanExecution, this, 
                BDIPlanExecution::Request().ABORT, current_plan_));
            thr->detach();
        }
    }

    /*
        Wrapper for calling addDesire with just desire to added (where not linked to any other desires)
        N.B see addDesire(const ManagedDesire mdAdd, const optional<ManagedDesire> necessaryForMd)
        for further explanations
    */
    bool addDesire(const ManagedDesire mdAdd)
    {
        return addDesire(mdAdd, std::nullopt, "");
    }

    /*
        Add desire @mdAdd to desire_set_ (if there is not yet)
        add counters for invalid desire and aborted plan in respective maps (both init to zero)
        @necessaryForMd is another ManagedDesire which can be present if @mdAdd is necessary 
        for the fulfillment of it (e.g. @mdAdd is derived from preconditions or context)
        @necessaryForMd value (if exists) has to be already in the desire_set_
    */
    bool addDesire(ManagedDesire mdAdd, optional<ManagedDesire> necessaryForMD, const string& suffixDesireGroup)
    {
        bool added = false;
        mtx_add_del_.lock();
            added = addDesireCS(mdAdd, necessaryForMD, suffixDesireGroup);
        mtx_add_del_.unlock();
        return added;
    }

    /*
        Add desire Critical Section (to be executed AFTER having acquired mtx_add_del_.lock())

        Add desire @mdAdd to desire_set_ (if there is not yet)
        add counters for invalid desire and aborted plan in respective maps (both init to zero)
        @necessaryForMd is another ManagedDesire which can be present if @mdAdd is necessary 
        for the fulfillment of it (e.g. @mdAdd is derived from preconditions or context)
        @necessaryForMd value (if exists) has to be already in the desire_set_
    */
    bool addDesireCS(ManagedDesire mdAdd, optional<ManagedDesire> necessaryForMD, const string& suffixDesireGroup)
    {
        if(mtx_add_del_.try_lock())
        {   
            //if acquired, it means we were not in CS, release it and return false -> operation not valid
            mtx_add_del_.unlock();
            return false;
        }
        
        bool added = false;
        if(computed_plan_desire_map_.count(mdAdd.getName())==0 && aborted_plan_desire_map_.count(mdAdd.getName())==0 && 
                desire_set_.count(mdAdd)==0)//desire already there (or diff. desire but with same name identifier)
        {
            if(necessaryForMD.has_value() && desire_set_.count(necessaryForMD.value())==1)
            {
                // @mdAdd linked to another MGdesire for which it is necessary in order to grant its execution
                // put as @mdAdd's group mdAdd.name + suffix (suffix can be "_precondition"/"_context")
                mdAdd.setDesireGroup(necessaryForMD.value().getName() + suffixDesireGroup);
            }

            desire_set_.insert(mdAdd);
            computed_plan_desire_map_.insert(std::pair<string, int>(mdAdd.getName(), 0));//to count invalid goal computations and discard after x
            aborted_plan_desire_map_.insert(std::pair<string, int>(mdAdd.getName(), 0));//to count invalid goal computations and discard after x
            
            added = true;
        }

        return added;
    }
    
    /*
        Wrapper for delDesire with two args
    */
    bool delDesire(const ManagedDesire mdDel)
    {
        return delDesire(mdDel, false);//do not delete desires within the same group of mdDel
    }

    /*
        Del desire from desire_set if present (Access through lock!)
    */
    bool delDesire(const ManagedDesire mdDel, const bool& wipeSameGroup)
    {
        bool deleted = false;
        mtx_add_del_.lock();
            deleted = delDesireCS(mdDel, wipeSameGroup);
        mtx_add_del_.unlock();

        return deleted;
    }
    
    /*
        Del desire from desire_set CRITICAL SECTION (to be called after having acquired mtx_add_del_ lock)
        In Addition Deleting atomically all the desires within the same desire group
        if @wipeSameGroup equals to true
    */
    bool delDesireCS(const ManagedDesire mdDel, const bool& wipeSameGroup)
    {
        if(mtx_add_del_.try_lock())
        {   
            //if acquired, it means we were not in CS, release it and return false -> operation not valid
            mtx_add_del_.unlock();
            return false;
        }

        bool deleted = false;
        //erase values from desires map
        if(computed_plan_desire_map_.count(mdDel.getName()) > 0)
            computed_plan_desire_map_.erase(mdDel.getName());
        if(aborted_plan_desire_map_.count(mdDel.getName()) > 0)
            aborted_plan_desire_map_.erase(mdDel.getName());

        if(desire_set_.count(mdDel)!=0)
        {
            desire_set_.erase(desire_set_.find(mdDel));
            computed_plan_desire_map_.erase(mdDel.getName());
            deleted = true;
            //RCLCPP_INFO(this->get_logger(), "Desire \"" + mdDel.getName() + "\" removed!");//TODO remove when assured there is no bug in deletion
        }/*else
            RCLCPP_INFO(this->get_logger(), "Desire \"" + mdDel.getName() + "\" to be removed NOT found!");*/
        
        if(wipeSameGroup && mdDel.hasParent()) // desire being part of a group where you need to satisfy just one
            delDesireInGroupCS(mdDel.getDesireGroup()); // delete others in the same group

        return deleted;
    }

    /*
        Del desire group from desire_set CRITICAL SECTION (to be called after having acquired mtx_add_del_ lock)
        Deleting atomically all the desires within the same desire group
    */
    void delDesireInGroupCS(const string& desireGroup)
    {
        vector<ManagedDesire> toBeDiscarded;

        for(ManagedDesire md : desire_set_)
            if(md.getDesireGroup() == desireGroup)
                toBeDiscarded.push_back(md);
        
        for(ManagedDesire md : toBeDiscarded)
            delDesireCS(md, false);//you're already iterating over all the desires within the same group
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

    // hashmap for invalid desire or plan not computed counters (after x tries desire will be discarded)
    map<string, int> computed_plan_desire_map_;
    // hashmap for aborted plan desire map (plan aborted for that desire)
    map<string, int> aborted_plan_desire_map_;

    // current_plan_ in execution (could be none if the agent isn't doing anything)
    ManagedPlan current_plan_;
    // last plan execution info
    BDIPlanExecutionInfo current_plan_exec_info_;

    //mutex for sync when modifying desire_set
    mutex mtx_add_del_;
    //mutex to regulate abort -> trigger new one flow
    mutex mtx_abort_trigger_;
    //to sync between iteration in checkForSatisfiedDesires() && reschedule()
    mutex mtx_iter_dset_;

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
