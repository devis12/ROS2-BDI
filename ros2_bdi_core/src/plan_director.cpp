#include <cstdlib>
#include <optional>
#include <ctime>
#include <memory>
#include <thread>
#include <chrono>
 
#include <boost/algorithm/string.hpp>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/condition.hpp"
#include "ros2_bdi_interfaces/msg/conditions_conjunction.hpp"
#include "ros2_bdi_interfaces/msg/conditions_dnf.hpp"
#include "ros2_bdi_interfaces/msg/plan_sys2_state.hpp"
#include "ros2_bdi_interfaces/msg/bdi_action_execution_info.hpp"
#include "ros2_bdi_interfaces/msg/bdi_plan_execution_info.hpp"
#include "ros2_bdi_interfaces/srv/bdi_plan_execution.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedConditionsConjunction.hpp"
#include "ros2_bdi_utils/ManagedConditionsDNF.hpp"
#include "ros2_bdi_utils/ManagedPlan.hpp"
#include "ros2_bdi_utils/BDIFilter.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_COMM_ERRORS 16
#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"
#define NO_PLAN_INTERVAL 1000
#define PLAN_INTERVAL 250

using std::string;
using std::thread;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::chrono::high_resolution_clock;
using std::bind;
using std::placeholders::_1;
using std::placeholders::_2;
using std::optional;

using plansys2::ExecutorClient;
using plansys2_msgs::msg::ActionExecutionInfo;
using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

using ros2_bdi_interfaces::srv::BDIPlanExecution;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::Condition;
using ros2_bdi_interfaces::msg::ConditionsConjunction;
using ros2_bdi_interfaces::msg::ConditionsDNF;
using ros2_bdi_interfaces::msg::PlanSys2State;
using ros2_bdi_interfaces::msg::BDIActionExecutionInfo;
using ros2_bdi_interfaces::msg::BDIPlanExecutionInfo;

typedef enum {STARTING, READY, EXECUTING, PAUSE} StateType;                

class PlanDirector : public rclcpp::Node
{
public:
  PlanDirector()
  : rclcpp::Node("plan_director"), state_(STARTING)
  {
    psys2_comm_errors_ = 0;
    this->declare_parameter(PARAM_AGENT_ID, "agent0");
    this->declare_parameter(PARAM_DEBUG, true);

    //object to notify the absence of a current plan execution
    no_plan_msg_ = BDIPlanExecutionInfo();
    no_plan_msg_.target = Desire();
    no_plan_msg_.target.name = (ManagedPlan{}).getDesire().getName();
    no_plan_msg_.target.value = vector<Belief>();
    no_plan_msg_.actions = vector<PlanItem>();
    no_plan_msg_.current_time = 0.0f;
    no_plan_msg_.estimated_deadline = 0.0f;
  }

  /*
    Init to call at the start, after construction method, to get the node actually started
    initialing psys2 executor client instance, 
    retrieving agent_id_ (thus namespace)
    defining work timer
  */
  void init()
  { 
    //agent's namespace
    agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

    // initializing executor client for psys2
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
    qos_keep_all.keep_all();

    //Check for plansys2 active state flags init to false
    psys2_executor_active_ = false;
    //plansys2 nodes status subscriber (receive notification from plansys2_monitor node)
    plansys2_status_subscriber_ = this->create_subscription<PlanSys2State>(
                "plansys2_state", qos_keep_all,
                bind(&PlanDirector::callbackPsys2State, this, _1));
    
    //belief_set_subscriber_ 
    belief_set_subscriber_ = this->create_subscription<BeliefSet>(
                "belief_set", qos_keep_all,
                bind(&PlanDirector::updatedBeliefSet, this, _1));

    // init server for triggering new plan execution
    server_plan_exec_ = this->create_service<BDIPlanExecution>("plan_execution", 
        bind(&PlanDirector::handlePlanRequest, this, _1, _2));

    // set NO_PLAN as current_plan_ 
    setNoPlanMsg();
    // plan execution notification
    plan_exec_publisher_ = this->create_publisher<BDIPlanExecutionInfo>("plan_execution_info", 10);

    //loop to be called regularly to perform work (publish belief_set_, sync with plansys2 problem_expert node...)
    do_work_timer_ = this->create_wall_timer(
        milliseconds(NO_PLAN_INTERVAL),
        bind(&PlanDirector::step, this));

    RCLCPP_INFO(this->get_logger(), "Plan director node initialized");
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
            if(psys2_executor_active_){
                psys2_comm_errors_ = 0;
                setState(READY);
            }else{
                RCLCPP_ERROR(this->get_logger(), "PlanSys2 Executor still not active");
                psys2_comm_errors_++;
            }

            break;
        }

        case READY:
        {
            if(this->get_parameter(PARAM_DEBUG).as_bool())
                RCLCPP_INFO(this->get_logger(), "Ready to accept new plan to be executed");
            publishNoPlanExec();//notify node it is currently on idle, i.e. not executing any plan
            break;
        }

        case EXECUTING:
        {    
            //RCLCPP_INFO(this->get_logger(), "Checking plan execution");
            executingPlan();
            break;
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
        Check planExecution feedback from plansys2 executor + check for context conditions
    */
    void executingPlan()
    {
        counter_check_++; 
        checkPlanExecution(); // get feedback from executor
        checkContextConditions(); // check if context conditions are still valid and true -> abort otherwise
    }

    /*
        When in READY state, msg to publish in plan_execution_info to notify it 
        (i.e. notify you're not executing any plan)
    */
    void publishNoPlanExec()
    {
        auto current_plan_desire = current_plan_.getDesire();
        if(current_plan_.getBody().size() == 0 &&
                current_plan_desire.getName() == no_plan_msg_.target.name && current_plan_desire.getPriority() == 0.0f)
        {
            //no plan currently in execution -> proceeds notifying that
            plan_exec_publisher_->publish(no_plan_msg_);
        }
    }
    
    /*
       Received notification about PlanSys2 nodes state by plansys2 monitor node
    */
    void callbackPsys2State(const PlanSys2State::SharedPtr msg)
    {
        psys2_executor_active_ = msg->executor_active;
    }

    /*
        Currently executing no plan
    */
    bool executingNoPlan()
    {
        return state_ != EXECUTING && current_plan_.getBody().size() == 0 && current_plan_.getDesire().getName() == ManagedPlan{}.getDesire().getName();
    }

    /*
        Cancel current plan execution (if any) and information preserved in it
    */
    void cancelCurrentPlanExecution()
    {
        //cancel plan execution
        executor_client_->cancel_plan_execution();
        if(this->get_parameter(PARAM_DEBUG).as_bool())
            RCLCPP_INFO(this->get_logger(), "Aborted plan execution");
    }


    /*
        Start new plan execution -> true if correctly started
    */
    bool startPlanExecution(const ManagedPlan& mp)
    {
        // prepare plansys2 msg for plan execution
        Plan plan_to_execute = Plan();
        plan_to_execute.items = mp.getBody();

        // select current_plan_ which will start execution
        current_plan_ = mp;
        // current_plan_start_ = high_resolution_clock::now();//plan started now
        bool started = executor_client_->start_plan_execution(plan_to_execute);

        if(started)
        {
            setState(EXECUTING);//put node in executing state
            //reset value, so they can be set at the first action execution feedback
            first_ts_plan_sec = -1;//reset this value
            first_ts_plan_nanosec = -1;//reset this value
            
            counter_check_ = 0;//checks performed during this plan exec

            //WORK TIMER set to PLAN EXEC MODE (callback checks more frequent)
            resetWorkTimer(PLAN_INTERVAL);

            if(this->get_parameter(PARAM_DEBUG).as_bool())
            {
                string plan_string = "";
                for(PlanItem pi : plan_to_execute.items)
                    plan_string += pi.action + "\t" + std::to_string(pi.duration) + "\n";
                RCLCPP_INFO(this->get_logger(), "Started new plan execution:\n" + plan_string + "\n");
            }
            
        }

        return started;
    }


    // clear info about current plan execution
    void setNoPlanMsg()
    {
        current_plan_ = ManagedPlan{};
    }

    // redefine workint timer interval
    void resetWorkTimer(const int& ms)
    {
        if(!do_work_timer_->is_canceled())
            do_work_timer_->cancel();
        
        do_work_timer_ = this->create_wall_timer(
            milliseconds(ms),
            bind(&PlanDirector::step, this));
    }

    /*  
        Callback to handle the service request to trigger a new plan execution or abort the current one
    */
    void handlePlanRequest(const BDIPlanExecution::Request::SharedPtr request,
        const BDIPlanExecution::Response::SharedPtr response)
    {
        string req_action = (request->request == request->EXECUTE)? "execute" : "abort";
        RCLCPP_INFO(this->get_logger(), "Received request to " + req_action + " plan fulfilling desire: " + request->plan.desire.name);

        ManagedDesire mdPlan = ManagedDesire{request->plan.desire};
        ManagedConditionsDNF mdPlanPrecondition = ManagedConditionsDNF{request->plan.precondition};
        ManagedConditionsDNF mdPlanContext = ManagedConditionsDNF{request->plan.context};
        bool done = false;
        if(request->ABORT && state_ == EXECUTING)// plan requested to be aborted it's in execution
        {
            ManagedPlan mp_abort = ManagedPlan{mdPlan, request->plan.actions, mdPlanPrecondition, mdPlanContext};

            if(current_plan_ == mp_abort)//request to abort plan which is currently in execution
            {
                cancelCurrentPlanExecution();
                checkPlanExecution();// last plan check for this plan execution
                done = executingNoPlan();
            }
        }
        else if(request->EXECUTE && state_ == READY)//no plan currently in exec
        {
            ManagedPlan requestedPlan = ManagedPlan{mdPlan, request->plan.actions, mdPlanPrecondition, mdPlanContext};
            // verify precondition before actually trying triggering executor
            if(requestedPlan.getPrecondition().isSatisfied(belief_set_))
            {
                bool started = startPlanExecution(requestedPlan);
                done = started && state_ == EXECUTING;
                if(done)
                    checkPlanExecution();// 1st checkPlanExecution for this plan (if started)
            } 
        }

    
        response->success = done;
    }

    /*
        Plan currently in execution, monitor and publish the feedback of its development
    */
    void checkContextConditions()
    {
        if(!current_plan_.getContext().isSatisfied(belief_set_))
        {
            //need to abort current plan execution because context condition are not valid anymore
            if(this->get_parameter(PARAM_DEBUG).as_bool())
                RCLCPP_INFO(this->get_logger(), "Aborting current plan execution because context conditions are not satisfied");
            
            cancelCurrentPlanExecution();
            checkPlanExecution();//to publish aborting and notifying subscribers
        }else{
            if(counter_check_ % 4 == 0 && this->get_parameter(PARAM_DEBUG).as_bool())//print just every 4 checks
                RCLCPP_INFO(this->get_logger(), "Current plan execution can go on: at least a context condition clause is satisfied");
        }

    }
    
    /*
        Plan currently in execution, monitor and publish the feedback of its development
    */
    void checkPlanExecution()
    {
        //get feedback from plansys2 api
        auto feedback = executor_client_->getFeedBack();

        // msg to publish about the plan execution
        BDIPlanExecutionInfo planExecutionInfo = getPlanExecutionInfo(feedback);

        plan_exec_publisher_->publish(planExecutionInfo);

        if(planExecutionInfo.status != planExecutionInfo.RUNNING)
        {
            //in any case plan execution has stopped, so go back to printing out you're not executing any plan
            resetWorkTimer(NO_PLAN_INTERVAL);
            setNoPlanMsg();
            setState(READY);
            
            // ended run log 
            if(this->get_parameter(PARAM_DEBUG).as_bool()){
                string result_s =   ((planExecutionInfo.status == planExecutionInfo.SUCCESSFUL)?
                    "executed successfully" : "aborted");
                RCLCPP_INFO(this->get_logger(), "Plan " + result_s + ": READY to execute new plan now\n");    
            }
        }
    }

    BDIPlanExecutionInfo getPlanExecutionInfo(const ExecutorClient::ExecutePlan::Feedback& feedback)
    {
        // retrieve plan body (action with duration and planned start step by step as computed by the pddl planner)
        vector<PlanItem> current_plan_body = current_plan_.getBody();

        BDIPlanExecutionInfo planExecutionInfo = BDIPlanExecutionInfo();
        float start_time_s = -1.0, status_time_s = -1.0;

        //find executing action status
        for (int i=0; i<feedback.action_execution_status.size(); i++) {
            BDIActionExecutionInfo actionExecutionInfo = BDIActionExecutionInfo();
            ActionExecutionInfo psys2_action_feed = feedback.action_execution_status[i];        
            if(psys2_action_feed.status == psys2_action_feed.EXECUTING)
            {
                actionExecutionInfo.args = psys2_action_feed.arguments;
                actionExecutionInfo.index = getActionIndex(psys2_action_feed.action, psys2_action_feed.arguments);//retrieve action index (NOT i :-/)
                actionExecutionInfo.name = psys2_action_feed.action;

                if(first_ts_plan_sec < 0)//set just for first start timestamp captured in this plan exec (then always subtract from it)
                    first_ts_plan_sec = psys2_action_feed.start_stamp.sec;
                if(first_ts_plan_nanosec < 0)//set just for first start timestamp captured in this plan exec (then always subtract from it)
                    first_ts_plan_nanosec = psys2_action_feed.start_stamp.nanosec;

                // compute start time of this action with respect first timestamp of first action start timestamp
                start_time_s = fromTimeMsgToSeconds(psys2_action_feed.start_stamp.sec - first_ts_plan_sec,
                    psys2_action_feed.start_stamp.nanosec - first_ts_plan_nanosec);
                
                // planned start time for this action
                actionExecutionInfo.planned_start = current_plan_body[actionExecutionInfo.index].time;

                // actual start time of this action
                actionExecutionInfo.actual_start = start_time_s;
                
                // compute status time of this action with respect first timestamp of first action start timestamp
                status_time_s = fromTimeMsgToSeconds(psys2_action_feed.status_stamp.sec - first_ts_plan_sec,
                        psys2_action_feed.status_stamp.nanosec - first_ts_plan_nanosec); 
               
                // retrieve execution time as (status_timestamp - start_timestamp)
                actionExecutionInfo.exec_time = status_time_s - start_time_s; 

                //retrieve estimated duration for action from pddl domain
                actionExecutionInfo.duration = fromTimeMsgToSeconds(psys2_action_feed.duration.sec,  
                    psys2_action_feed.duration.nanosec);

                actionExecutionInfo.status = psys2_action_feed.completion;

                planExecutionInfo.executing.push_back(actionExecutionInfo);//add action execution info to plan execution info 
            }    
        }

        
        planExecutionInfo.target = current_plan_.getDesire().toDesire();
        planExecutionInfo.actions = current_plan_body;
        planExecutionInfo.estimated_deadline = current_plan_.getPlanDeadline();
        
        //current time ms computed with real clock (option left aside for now)
        //float current_time_ms =  (float) (std::chrono::duration<double, std::milli>(high_resolution_clock::now()-current_plan_start_).count()) / pow(10, 3);
        
        // current time s computed by difference from fist start ts of first action executed within the plan
        planExecutionInfo.current_time = (status_time_s >= 0.0f)? status_time_s : 0.0f;
        planExecutionInfo.status = getPlanExecutionStatus();

        return planExecutionInfo;
    }

    uint8_t getPlanExecutionStatus()
    {
        uint8_t result = BDIPlanExecutionInfo().RUNNING;
        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) //plan stopped
        {      
            if(executor_client_->getResult().value().success)//successful  run
                result = BDIPlanExecutionInfo().SUCCESSFUL;
            
            else //plan aborted
                result = BDIPlanExecutionInfo().ABORT;
        }   
        return result;
    }


    /*  
        Take time msg (int second, int nanosecond)
        transform it in float second
    */
    float fromTimeMsgToSeconds(const int& sec_ts, const int& nanosec_ts)
    {
        return sec_ts + 
                    (round((nanosec_ts) / pow(10, 6)) / pow(10, 3));
    }

    /*
        get index of action with given action_name & args in the vector<PlanItem> in current_plan_.body 
    */
    int getActionIndex(const string& action_name, const vector<string>& args)
    {
        int foundIndex = -1;
        // retrieve plan body (action with duration and planned start step by step as computed by the pddl planner)
        vector<PlanItem> current_plan_body = current_plan_.getBody();

        for(int i = 0; i < current_plan_body.size(); i++)
        {   
            string full_name_i = current_plan_body[i].action;// (action_name param1 param2 ...)
            if(full_name_i.length() > 0)
            {    
                if(full_name_i[0] == '(')
                    full_name_i = full_name_i.substr(1);//remove start parenthesis from action name

                if(full_name_i[full_name_i.length()-1] == ')')
                    full_name_i = full_name_i.substr(0, full_name_i.length()-1);//remove end parenthesis from action name
            }   

            vector<string> action_params_i;
            boost::split(action_params_i, full_name_i, [](char c){return c == ' ';});//split string
            if(action_params_i[0] == action_name && action_params_i.size()-1 == args.size())//same name && same num of args 
            {
                bool matchingParams = true; //true if all the arg of the action match
                for(int j = 1; matchingParams && j < action_params_i.size(); j++)
                    if(action_params_i[j] != args[j-1])
                        matchingParams = false;
                
                if(matchingParams)// this is the correct action we were looking for -> return i
                    return i;
            }
                
        }
        return foundIndex;
    }

    /*
        The belief set has been updated
    */
    void updatedBeliefSet(const BeliefSet::SharedPtr msg)
    {
        belief_set_ = BDIFilter::extractMGBeliefs(msg->value);
    }

    // internal state of the node
    StateType state_;
    
    // agent id that defines the namespace in which the node operates
    string agent_id_;

    // timer to trigger callback to perform main loop of work regularly
    rclcpp::TimerBase::SharedPtr do_work_timer_;

    // counter of communication errors with plansys2
    int psys2_comm_errors_;
    // executor client contacting psys2 for the execution of a plan, then receiving feedback for it 
    shared_ptr<ExecutorClient> executor_client_;

    // flag to denote if plansys2 executor appears to be active
    bool psys2_executor_active_;
    // plansys2 node status monitor subscription
    rclcpp::Subscription<PlanSys2State>::SharedPtr plansys2_status_subscriber_;

    // current_plan_ in execution (could be none if the agent isn't doing anything)
    ManagedPlan current_plan_;
    // # checks performed during the current plan exec
    int counter_check_;
    // time at which plan started (NOT DOING this anymore -> using first start_ts from first action executed in plan)
    //high_resolution_clock::time_point current_plan_start_;
    // msg to notify the idle-ready state, i.e. no current plan execution, but ready to do it
    BDIPlanExecutionInfo no_plan_msg_;

    // current belief set (in order to check precondition && context condition)
    set<ManagedBelief> belief_set_;
    // belief set subscriber
    rclcpp::Subscription<BeliefSet>::SharedPtr belief_set_subscriber_;//belief set sub.

    // record first timestamp in sec of the current plan execution (to subtract from it)
    int first_ts_plan_sec;
    int first_ts_plan_nanosec;

    // notification about the current plan execution
    rclcpp::Publisher<BDIPlanExecutionInfo>::SharedPtr plan_exec_publisher_;//belief set publisher

    // trigger plan execution/abortion service
    rclcpp::Service<BDIPlanExecution>::SharedPtr server_plan_exec_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanDirector>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
