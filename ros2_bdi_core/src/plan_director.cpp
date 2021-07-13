#include <cstdlib>
#include <optional>
#include <ctime>
#include <memory>
#include <thread>
#include <chrono>
 
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_executor/ExecutorClient.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/bdi_action_execution_info.hpp"
#include "ros2_bdi_interfaces/msg/bdi_plan_execution_info.hpp"
#include "ros2_bdi_interfaces/srv/bdi_plan_execution.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/ManagedPlan.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_COMM_ERRORS 16
#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"

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
using ros2_bdi_interfaces::msg::Desire;
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

    psys2_executor_up_ = false;

    //object to notify the absence of a current plan execution
    no_plan_msg_ = BDIPlanExecutionInfo();
    no_plan_msg_.target = Desire();
    no_plan_msg_.target.name = "NO_PLAN";
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

    // init server for triggering new plan execution
    server_plan_exec_ = this->create_service<BDIPlanExecution>("plan_execution", 
        bind(&PlanDirector::handlePlanRequest, this, _1, _2));

    // plan execution notification
    plan_exec_publisher_ = this->create_publisher<BDIPlanExecutionInfo>("plan_execution_info", 10);

    //loop to be called regularly to perform work (publish belief_set_, sync with plansys2 problem_expert node...)
    do_work_timer_ = this->create_wall_timer(
        milliseconds(500),
        bind(&PlanDirector::step, this));
    
    //check if executor is still up (cancel timer if plansys2 seems up)
    check_executor_timer_ = this->create_wall_timer(
        milliseconds(2000),
        bind(&PlanDirector::executorUp, this));

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
            if(psys2_executor_up_){
                this->check_executor_timer_->cancel();
                psys2_comm_errors_ = 0;
                setState(READY);
            }else{
                executorUp();
                psys2_executor_up_ = false;
                RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with PlanSys2 executor");
                psys2_comm_errors_++;
            }

            break;
        }

        case READY:
        {
            //RCLCPP_INFO(this->get_logger(), "Ready to accept new plan to be executed");
            publishNoPlanExec();//notify node it is currently on idle, i.e. not executing any plan
            break;
        }

        case EXECUTING:
        {    
            //RCLCPP_INFO(this->get_logger(), "Checking plan execution");
            checkPlanExecution();
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
        When in READY state, msg to publish in plan_execution_info to notify it 
        (i.e. notify you're not executing any plan)
    */
    void publishNoPlanExec()
    {
        if(current_plan_.body_.size() == 0 &&
                current_plan_.desire_.name_ == "" && current_plan_.desire_.priority_ == 0.0f)
        {
            //no plan currently in execution -> proceeds notifying that
            plan_exec_publisher_->publish(no_plan_msg_);
        }
    }

    void executorUp()
    {
        shared_ptr<thread> thr = 
                std::make_shared<thread>(bind(&PlanDirector::checkExecutorUp, this));
        thr->detach();
    }

     /*
        Check for the presence of two topic to decide if the plansys2 executor is alive
    */
    void checkExecutorUp()
    {
        //check for service to be up
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_ = 
            this->create_client<lifecycle_msgs::srv::GetState>("executor/get_state");
        if(!client_->wait_for_service(std::chrono::seconds(1))){
            //service seems not even present, just return false
            RCLCPP_WARN(this->get_logger(), "executor/get_state server does not appear to be up");
            return;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        
        auto future = client_->async_send_request(request);

        try{
            auto response = future.get();
            psys2_executor_up_ = response->current_state.id == 3 && response->current_state.label == "active";
        }catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Response error in executor/get_state");
        }
    }

    /*
        Cancel current plan execution (if any) and information preserved in it
    */
    void cancelCurrentPlanExecution()
    {
        //cancel plan execution
        executor_client_->cancel_plan_execution();

        //clear info about current plan execution
        setNoPlanMsg();
      
    }

    //clear info about current plan execution
    void setNoPlanMsg()
    {
        current_plan_ = ManagedPlan{};
        current_plan_.desire_.name_ = no_plan_msg_.target.name;
    }

    /*  
        Callback to handle the service request to trigger a new plan execution or abort the current one
    */
    void handlePlanRequest(const BDIPlanExecution::Request::SharedPtr request,
        const BDIPlanExecution::Response::SharedPtr response)
    {
        string req_action = (request->request == request->EXECUTE)? "execute" : "abort";
        RCLCPP_INFO(this->get_logger(), "Received request to %s plan fulfilling desire: ", req_action, request->plan.desire.name);

        bool done = false;
        if(request->ABORT && state_ == EXECUTING)// plan requested to be aborted it's in execution
        {
            ManagedDesire md_abort = ManagedDesire{request->plan.desire};
            ManagedPlan mp_abort = ManagedPlan{md_abort, request->plan.actions};
            if(current_plan_ == mp_abort)//request to abort plan which is currently in execution
            {
                cancelCurrentPlanExecution();
                setState(READY);//put node in ready state (ready to receive plan to execute, not executing any plan right now)
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                    RCLCPP_INFO(this->get_logger(), "Aborted plan execution");
                done = true;
            }
        }
        else if(request->EXECUTE && state_ == READY)//no plan currently in exec
        {
            //start plan execution
            Plan plan_to_execute = Plan();
            plan_to_execute.items = request->plan.actions;
            if(executor_client_->start_plan_execution(plan_to_execute))
            {
                current_plan_ = ManagedPlan{request->plan.desire, request->plan.actions};
                current_plan_start_ = high_resolution_clock::now();//plan started now
                setState(EXECUTING);//put node in executing state
                
                if(this->get_parameter(PARAM_DEBUG).as_bool())
                {
                    string plan_string = "";
                    for(PlanItem pi : plan_to_execute.items)
                        plan_string += pi.action + "\t" + std::to_string(pi.duration) + "\n";
                    RCLCPP_INFO(this->get_logger(), "Started new plan execution:\n" + plan_string + "\n");
                }
                    
                done = true;
            } 
        }

        response->success = done;
    }

    /*
        Plan currently in execution, monitor and publish the feedback of its development
    */
    void checkPlanExecution()
    {

        BDIActionExecutionInfo actionExecutionInfo = BDIActionExecutionInfo();
        auto feedback = executor_client_->getFeedBack();

        //find executing action status
        for (int i=0; i<feedback.action_execution_status.size(); i++) {
            ActionExecutionInfo psys2_action_feed = feedback.action_execution_status[i];        
            if(psys2_action_feed.EXECUTING && psys2_action_feed.completion > 0.0 && psys2_action_feed.completion < 1.0)
            {
                actionExecutionInfo.args = psys2_action_feed.arguments;
                actionExecutionInfo.index = i;
                actionExecutionInfo.name = psys2_action_feed.action;
                actionExecutionInfo.started = (float)psys2_action_feed.start_stamp.sec + 
                    (((float)psys2_action_feed.start_stamp.nanosec) / pow(10,9));
                actionExecutionInfo.status = psys2_action_feed.completion;
                break;
            }    
        }

        BDIPlanExecutionInfo planExecutionInfo = BDIPlanExecutionInfo();
        planExecutionInfo.target = current_plan_.desire_.toDesire();
        planExecutionInfo.actions = current_plan_.body_;
        planExecutionInfo.executing = actionExecutionInfo;
        planExecutionInfo.estimated_deadline = current_plan_.plan_deadline_;
        planExecutionInfo.current_time = (float)
            (std::chrono::duration<double, std::milli>(high_resolution_clock::now()-current_plan_start_).count());
        planExecutionInfo.status = planExecutionInfo.RUNNING;
        
        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) //plan stopped
        {      
            if(executor_client_->getResult().value().success)//successful  run
            {
                planExecutionInfo.status = planExecutionInfo.SUCCESSFUL;
                //not executing any plan now
                setNoPlanMsg();
                setState(READY);
            }
            
            else //plan aborted
                planExecutionInfo.status =  planExecutionInfo.ABORT;
        }   

        plan_exec_publisher_->publish(planExecutionInfo);
    }

    // internal state of the node
    StateType state_;
    
    // agent id that defines the namespace in which the node operates
    string agent_id_;

    // timer to trigger callback to perform main loop of work regularly
    rclcpp::TimerBase::SharedPtr do_work_timer_;
    // timer to trigger check for executor to be up
    rclcpp::TimerBase::SharedPtr check_executor_timer_;

    // counter of communication errors with plansys2
    int psys2_comm_errors_;
    // executor client contacting psys2 for the execution of a plan, then receiving feedback for it 
    shared_ptr<ExecutorClient> executor_client_;

    bool psys2_executor_up_;
    // current_plan_ in execution (could be none if the agent isn't doing anything)
    ManagedPlan current_plan_;
    // time at which plan started
    high_resolution_clock::time_point current_plan_start_;
    // msg to notify the idle-ready state, i.e. no current plan execution, but ready to do it
    BDIPlanExecutionInfo no_plan_msg_;

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
