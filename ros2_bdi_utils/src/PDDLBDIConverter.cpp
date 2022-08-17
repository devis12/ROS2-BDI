#include "ros2_bdi_utils/PDDLBDIConverter.hpp"

#include "ros2_bdi_utils/PDDLUtils.hpp"

#include <boost/algorithm/string.hpp>

using std::string;
using std::vector;

using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;

using plansys2_msgs::msg::PlanItem;
using plansys2_msgs::msg::ActionExecutionInfo;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::BDIActionExecutionInfo;

namespace PDDLBDIConverter
{

  /*
    Convert PlanSys2 PDDL Instance to ROS2-BDI Belief
  */
  Belief convertPDDLInstance(const Instance instance)
  {
    Belief b = Belief();
    
    b.name = instance.name;
    b.pddl_type = Belief().INSTANCE_TYPE;

    vector<string> params = vector<string>({instance.type});
    b.params = params;
    
    b.value = 0.0f;// has NO meaning in Instance type
    
    return b;
  }

  /*
    Convert PlanSys2 PDDL Instances to ROS2-BDI Beliefs
  */
  vector<Belief> convertPDDLInstances(const vector<Instance> instances)
  {
    vector<Belief> beliefs = vector<Belief>();
    for(auto ins : instances)
      beliefs.push_back(convertPDDLInstance(ins));
    return beliefs;
  }

   /*
    Convert PlanSys2 PDDL Predicate to ROS2-BDI Belief
  */
  Belief convertPDDLPredicate(const Predicate predicate)
  {
    Belief b = Belief();
    
    b.name = predicate.name;
    b.pddl_type = Belief().PREDICATE_TYPE;

    vector<string> params = vector<string>();
    for(auto p : predicate.parameters)
        params.push_back(p.name);
    b.params = params;
    
    b.value = 0.0f;// has NO meaning in Predicate type
    
    return b;
  }


  /*
    Convert PlanSys2 PDDL Predicates to ROS2-BDI Beliefs
  */
  vector<Belief> convertPDDLPredicates(const vector<Predicate> predicates){
    vector<Belief> beliefs = vector<Belief>();
    for(auto p : predicates)
      beliefs.push_back(convertPDDLPredicate(p));
    return beliefs;
  }


    /*
    Convert PlanSys2 PDDL Function to ROS2-BDI Belief
  */
  Belief convertPDDLFunction(const Function function)
  {
    Belief b = Belief();
    
    b.name = function.name;
    b.pddl_type = Belief().FUNCTION_TYPE;

    vector<string> params = vector<string>();
    for(auto p : function.parameters)
        params.push_back(p.name);
    b.params = params;
    
    b.value = function.value;
    
    return b;
  }

  /*
    Convert PlanSys2 PDDL Functions to ROS2-BDI Beliefs
  */
  vector<Belief> convertPDDLFunctions(const vector<Function> functions){
    vector<Belief> beliefs = vector<Belief>();
    for(auto f : functions)
      beliefs.push_back(convertPDDLFunction(f));
    return beliefs;
  }

  typedef enum {ADD_NANO_SEC, DEL_NANO_SEC} OpSign;

  /*
    builtin_interfaces/msg/Time representation to float conversion
  */
  float fromTimeToFloat(const int& sec_ts, const unsigned int& nanosec_ts, OpSign nanosec_op = ADD_NANO_SEC)
  {
    if(nanosec_op == ADD_NANO_SEC)
      return sec_ts + (round((nanosec_ts) / pow(10, 6)) / pow(10, 3));
    else
      return sec_ts - (round((nanosec_ts) / pow(10, 6)) / pow(10, 3));
  }

  /*  
    Take PlanSys2 time msg (int second, uint nanosecond) wrt. interface builtin_interfaces/msg/Time
    transform it in pure float second representing the relative time from the plan start marked as 
    the first action start timestamp
  */
  float computeRelativeTime(
    const int& current_sec_ts, const unsigned int& current_nanosec_ts, 
    const int& start_sec_ts, const unsigned int& start_nanosec_ts)
  { 
    int rel_sec = (current_sec_ts >= start_sec_ts)? current_sec_ts - start_sec_ts : 0;
    
    unsigned int rel_nanosec = (current_nanosec_ts >= start_nanosec_ts)? 
      current_nanosec_ts - start_nanosec_ts : 
      start_nanosec_ts - current_nanosec_ts;
      
    if(current_nanosec_ts >= start_nanosec_ts)
      return fromTimeToFloat(rel_sec, rel_nanosec, ADD_NANO_SEC);
    else
      return fromTimeToFloat(rel_sec, rel_nanosec, DEL_NANO_SEC);
  }

  /*
    get index of action with given action_name & args in the vector<PlanItem> in current_plan_.body 
    action_full_name is in the form "(a1 p1 p2 p3):timex1000"
  */
  int getActionIndex(const vector<PlanItem>& current_plan_body, const string& action_full_name)
  {
      int timex1000 = std::stoi(action_full_name.substr(action_full_name.find_last_of(":")+1));
      string action_full_no_time = action_full_name.substr(0, action_full_name.find_first_of(":"));// (action_name param1 param2 ...)
      
      for(int i = 0; i < current_plan_body.size(); i++)
      {   
          int plannedStartTimex1000 = (int) (current_plan_body[i].time * 1000);

          if(timex1000 == plannedStartTimex1000 && action_full_no_time == current_plan_body[i].action)//same time of start
            return i;
              
      }
      return -1;
  }

  /*
    get index of action with given action_name & args in the vector<ActionExecutionInfo> in psys2_plan_exec_info
    action_full_name is in the form "(a1 p1 p2 p3):timex1000"
  */
  int getActionIndex(const vector<ActionExecutionInfo>& psys2_plan_exec_info, const string& action_full_name)
  {   
      for(int i = 0; i < psys2_plan_exec_info.size(); i++)
      {   
          if(psys2_plan_exec_info[i].action_full_name == action_full_name)//same name, args and time of start (all wrapped in string action_full_name)
            return i;
      }
      return -1;
  }

  /*
    Get from plansys2 action feedback BDIActionExecutionInfo status
  */
  int16_t getBDIActionExecutionStatus(const ActionExecutionInfo& psys2_action_feed)
  {
    BDIActionExecutionInfo bdi_ai = BDIActionExecutionInfo();
    int16_t res = bdi_ai.UNKNOWN;
    //switch(psys2_action_feed.status)
    //{
      if(psys2_action_feed.status == psys2_action_feed.CANCELLED) 
        res = bdi_ai.FAILED;
        //break;
      else if(psys2_action_feed.status == psys2_action_feed.EXECUTING) 
        res = bdi_ai.RUNNING;
        //break;
      else if(psys2_action_feed.status == psys2_action_feed.FAILED) 
        res = bdi_ai.FAILED;
        //break;
      else if(psys2_action_feed.status == psys2_action_feed.NOT_EXECUTED) 
        res = bdi_ai.WAITING;
        //break;
      else if(psys2_action_feed.status == psys2_action_feed.SUCCEEDED) 
        res = bdi_ai.SUCCESSFUL;
        //break;
      else
        res = bdi_ai.UNKNOWN;
    //}
    return res;
  }

  /*
    Build a BDIActionExecutionInfo from the corresponding PlanSys2 ActionExecutionInfo
    PlanSys2 plan body is needed too (PlanItem array) to get the index of the executing action within it
  */
  BDIActionExecutionInfo buildBDIActionExecutionInfo(

    const std::optional<ActionExecutionInfo>& psys2_action_feed_opt, 
    const vector<PlanItem>& current_plan_body,
    const int& action_index, 
    const int& first_ts_plan_sec, const unsigned int& first_ts_plan_nanosec)
  {
    ActionExecutionInfo psys2_action_feed = psys2_action_feed_opt.has_value()? psys2_action_feed_opt.value() : ActionExecutionInfo();
    
    BDIActionExecutionInfo bdiActionExecutionInfo = BDIActionExecutionInfo();
    vector<string> plan_item_elems = PDDLUtils::extractPlanItemActionElements(current_plan_body[action_index].action);

    //assign index
    bdiActionExecutionInfo.index = action_index;
    //assign action's name
    bdiActionExecutionInfo.name = plan_item_elems[0];
    //assign action's args
    for(int i = 1; i<plan_item_elems.size(); i++)
      bdiActionExecutionInfo.args.push_back(plan_item_elems[i]);

    bdiActionExecutionInfo.wait_action_indexes = std::vector<short int>();
    if(psys2_action_feed_opt.has_value())
    {
      for(auto waitAction : psys2_action_feed.waiting_actions)
        bdiActionExecutionInfo.wait_action_indexes.push_back(getActionIndex(current_plan_body, waitAction));
    }

    if(psys2_action_feed_opt.has_value() && first_ts_plan_sec >= 0 && psys2_action_feed.status != psys2_action_feed.NOT_EXECUTED)
    {
      // compute start time of this action with respect first timestamp of first action start timestamp
      float start_time_s = computeRelativeTime(psys2_action_feed.start_stamp.sec, psys2_action_feed.start_stamp.nanosec,
                              first_ts_plan_sec, first_ts_plan_nanosec);
      // actual start time of this action
      bdiActionExecutionInfo.actual_start = start_time_s;

      // compute status time of this action with respect first timestamp of first action start timestamp
      float status_time_s = computeRelativeTime(psys2_action_feed.status_stamp.sec, psys2_action_feed.status_stamp.nanosec,
                                first_ts_plan_sec, first_ts_plan_nanosec); 
      
      // retrieve execution time as (status_timestamp - start_timestamp)
      bdiActionExecutionInfo.exec_time = status_time_s - start_time_s; 
    }
    else
    { 
      // still having no info
      bdiActionExecutionInfo.actual_start = 0.0f;
      bdiActionExecutionInfo.exec_time = 0.0f;
    }

    // planned start time for this action
    bdiActionExecutionInfo.planned_start = current_plan_body[action_index].time;

    //retrieve estimated duration for action from pddl domain
    bdiActionExecutionInfo.duration = current_plan_body[action_index].duration;

    bdiActionExecutionInfo.progress = psys2_action_feed_opt.has_value()? psys2_action_feed.completion : 0.0f;

    bdiActionExecutionInfo.status = psys2_action_feed_opt.has_value()? getBDIActionExecutionStatus(psys2_action_feed) : bdiActionExecutionInfo.UNKNOWN;

    return bdiActionExecutionInfo;
  }

}
