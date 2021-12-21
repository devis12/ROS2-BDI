#include "ros2_bdi_utils/PDDLBDIConverter.hpp"

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
  */
  int getActionIndex(const vector<PlanItem>& current_plan_body, const string& action_name, const vector<string>& args)
  {
      int foundIndex = -1;

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
    Build a BDIActionExecutionInfo from the corresponding PlanSys2 ActionExecutionInfo
    PlanSys2 plan body is needed too (PlanItem array) to get the index of the executing action within it
  */
  BDIActionExecutionInfo buildBDIActionExecutionInfo(

    const ActionExecutionInfo& psys2_action_feed, 
    const vector<PlanItem>& current_plan_body,
    const int& first_ts_plan_sec, const unsigned int& first_ts_plan_nanosec)
  {
    BDIActionExecutionInfo bdiActionExecutionInfo = BDIActionExecutionInfo();
    bdiActionExecutionInfo.args = psys2_action_feed.arguments;
    bdiActionExecutionInfo.index = getActionIndex(current_plan_body, psys2_action_feed.action, psys2_action_feed.arguments);//retrieve action index within plan body (NOT i :-/)
    bdiActionExecutionInfo.name = psys2_action_feed.action;

    // compute start time of this action with respect first timestamp of first action start timestamp
    float start_time_s = computeRelativeTime(psys2_action_feed.start_stamp.sec, psys2_action_feed.start_stamp.nanosec,
                            first_ts_plan_sec, first_ts_plan_nanosec);

    // planned start time for this action
    bdiActionExecutionInfo.planned_start = current_plan_body[bdiActionExecutionInfo.index].time;

    // actual start time of this action
    bdiActionExecutionInfo.actual_start = start_time_s;
    
    // compute status time of this action with respect first timestamp of first action start timestamp
    float status_time_s = computeRelativeTime(psys2_action_feed.status_stamp.sec, psys2_action_feed.status_stamp.nanosec,
                              first_ts_plan_sec, first_ts_plan_nanosec); 
    
    // retrieve execution time as (status_timestamp - start_timestamp)
    bdiActionExecutionInfo.exec_time = status_time_s - start_time_s; 

    //retrieve estimated duration for action from pddl domain
    bdiActionExecutionInfo.duration = fromTimeToFloat(psys2_action_feed.duration.sec, psys2_action_feed.duration.nanosec);

    bdiActionExecutionInfo.progress = psys2_action_feed.completion;

    return bdiActionExecutionInfo;
  }

}
