#include "ros2_bdi_utils/ManagedPlan.hpp"

#include "ros2_bdi_utils/PDDLUtils.hpp"

#include "ros2_bdi_interfaces/msg/desire.hpp"

#include <boost/algorithm/string.hpp>

using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::BDIPlan;
using ros2_bdi_interfaces::msg::BDIActionExecutionInfo;

using BDIManaged::ManagedPlan;

using std::string;
using std::vector;

#define NO_PLAN "NO_PLAN"

string ManagedPlan::computeActionFullName(BDIActionExecutionInfo action_exec)
{
    string full_name = "("+action_exec.name;
    
    for(string arg : action_exec.args)
        full_name += " " + arg;
    full_name += ")";

    return full_name;
}

vector<BDIActionExecutionInfo> ManagedPlan::computeActionsExecInfo(vector<PlanItem> plan_items)
{

    vector<BDIActionExecutionInfo> actions_exec_info = vector<BDIActionExecutionInfo>();

    for(PlanItem pi : plan_items)
    {
        BDIActionExecutionInfo bdi_ai = BDIActionExecutionInfo();

        vector<string> tokens = PDDLUtils::extractPlanItemActionElements(pi.action);
        bdi_ai.name = tokens[0];

        for(uint16_t i = 1; i<tokens.size(); i++)
            bdi_ai.args.push_back(tokens[i]);

        bdi_ai.status = bdi_ai.UNKNOWN;
        bdi_ai.actual_start = 0.0f;
        bdi_ai.planned_start = pi.time;
        bdi_ai.duration = pi.duration;
        bdi_ai.progress = 0.0f;
        bdi_ai.exec_time = 0.0f;

        actions_exec_info.push_back(bdi_ai);
    }

    return actions_exec_info;
}

ManagedPlan::ManagedPlan():
    actions_exec_info_(vector<BDIActionExecutionInfo>()),
    precondition_(ManagedConditionsDNF{}),
    context_(ManagedConditionsDNF{}),
    planned_deadline_(0.0f)
    {
        Desire d = Desire{};
        d.name = NO_PLAN;
        final_target_ = std::make_shared<ManagedDesire>(d);
        plan_target_ = final_target_;
    }

ManagedPlan::ManagedPlan(const int16_t& plan_index, const ManagedDesire& md, const vector<PlanItem>& planitems):
    planqueue_index_(plan_index),
    actions_exec_info_(computeActionsExecInfo(planitems)),
    precondition_(ManagedConditionsDNF{}),
    context_(ManagedConditionsDNF{})
    {   
        final_target_ = std::make_shared<ManagedDesire>(md);
        plan_target_ = final_target_;
        planned_deadline_ = computePlannedDeadline();
    }

ManagedPlan::ManagedPlan(const int16_t& plan_index, const ManagedDesire& md, const vector<PlanItem>& planitems, 
    const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context):
    planqueue_index_(plan_index),
    actions_exec_info_(computeActionsExecInfo(planitems)),
    precondition_(precondition),
    context_(context)
    {   
        final_target_ = std::make_shared<ManagedDesire>(md);
        plan_target_ = final_target_;
        planned_deadline_ = computePlannedDeadline();
    }

ManagedPlan::ManagedPlan(const int16_t& plan_index, const ManagedDesire& finalDesire, const ManagedDesire& intermediateDesire,
    const vector<PlanItem>& planitems, const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context):
    planqueue_index_(plan_index),
    actions_exec_info_(computeActionsExecInfo(planitems)),
    precondition_(precondition),
    context_(context)
    {   
        final_target_ = std::make_shared<ManagedDesire>(finalDesire);
        plan_target_ = std::make_shared<ManagedDesire>(intermediateDesire);;
        planned_deadline_ = computePlannedDeadline();
    }

Plan ManagedPlan::toPsys2Plan() const
{
    Plan p = Plan();
    p.plan_index = planqueue_index_;
    p.items =  vector<PlanItem>();
    for(BDIActionExecutionInfo bdi_ai : actions_exec_info_)
    {
        PlanItem pi = PlanItem();
        pi.time = bdi_ai.planned_start;
        pi.duration = bdi_ai.duration;
        pi.action = ManagedPlan::computeActionFullName(bdi_ai);
        pi.committed = true;
        p.items.push_back(pi);
    }
    return p;
}

BDIPlan ManagedPlan::toPlan() const
{
    BDIPlan p = BDIPlan();
    p.target = plan_target_->toDesire();
    p.psys2_plan = toPsys2Plan();
    p.precondition = precondition_.toConditionsDNF();
    p.context = context_.toConditionsDNF();

    return p;
}


float ManagedPlan::computePlannedDeadline()
{
    float deadline = 0.0f;
    // you cannot compute the sum of all duration, because not all plans are 
    // linear sequence of actions (i.e. actions can start in group and/or actions
    // can start when other actions during plan exec. has not finished yet)
    for(BDIActionExecutionInfo bdi_ai : actions_exec_info_)
        deadline = std::max(deadline, bdi_ai.planned_start + bdi_ai.duration);
    return deadline;
}

float ManagedPlan::computeUpdatedEndTime(const BDIActionExecutionInfo& bdi_ai)
{
    if(bdi_ai.status == bdi_ai.RUNNING)
    {   
        float updated_exec_time_lp = bdi_ai.exec_time / bdi_ai.progress;//TODO this assume linear progress, but one can specify its own policy: design this better in future iteration
        return bdi_ai.actual_start + updated_exec_time_lp;
    }
    else if (bdi_ai.status == bdi_ai.SUCCESSFUL || bdi_ai.status == bdi_ai.FAILED)//I know exactly how much they took
        return bdi_ai.actual_start + bdi_ai.exec_time;
        
    else if(bdi_ai.status == bdi_ai.UNKNOWN || bdi_ai.status == bdi_ai.WAITING && bdi_ai.wait_action_indexes.size() == 0)//waiting or unknown status, but in both cases non waiting anyone -> just return the planned end
        return bdi_ai.planned_start + bdi_ai.duration;

    else
    {
        float max_end_time = 0.0f;
        for(int i = 0; i < bdi_ai.wait_action_indexes.size(); i++)
        {
            BDIActionExecutionInfo bdi_ai_to_be_waited = actions_exec_info_[bdi_ai.wait_action_indexes[i]];
            max_end_time = std::max(max_end_time, computeUpdatedEndTime(bdi_ai_to_be_waited) + bdi_ai.duration); 
        }
        return max_end_time;
    }
}

float ManagedPlan::getUpdatedEstimatedDeadline()
{
    float deadline = 0.0f;
    
    // you cannot compute the sum of all duration, because not all plans are 
    // linear sequence of actions (i.e. actions can start in group and/or actions
    // can start when other actions during plan exec. has not finished yet)
    for(BDIActionExecutionInfo bdi_ai : actions_exec_info_)
        deadline = std::max(deadline, computeUpdatedEndTime(bdi_ai)); 

    return deadline;
}


/*
    Try to parse an array of psys2 plan items from a string, format is the following
    
    [start_time](action)[duration]\n[start_time](action)[duration]
*/
std::optional<vector<PlanItem>> ManagedPlan::parsePsys2PlanMsg(string plan_msg)
{
    if(plan_msg.length() == 0)
        return std::nullopt;
    else
    {
        vector<PlanItem> p_items;
        vector<string> s_pitems;
        boost::split(s_pitems, plan_msg, [](char c){return c == '\n';});//split string
        for(string s_pitem : s_pitems)
        {   
            //init values for parsing pitem
            int i = 0;

            float start_time = -1.0f;
            string action = "";
            float duration = -1.0f;

            //retrieve start time
            string s_start_time = "";
            for(; i<s_pitem.size() && s_pitem.at(i) != ']'; i++)
                if(s_pitem.at(i) != '[' && s_pitem.at(i) != ']' )
                    s_start_time += s_pitem.at(i);
            start_time = atof(s_start_time.c_str());

            //retrieve action
            string s_action = "";
            for(; i<s_pitem.size() && s_pitem.at(i) != '['; i++)
                if(s_pitem.at(i) != '[' && s_pitem.at(i) != ']' )
                    s_action += s_pitem.at(i);

            //retrieve duration
            string s_duration = "";
            for(; i<s_pitem.size() && s_pitem.at(i) != ']'; i++)
                if(s_pitem.at(i) != '[' && s_pitem.at(i) != ']' )
                    s_duration += s_pitem.at(i);
            duration = atof(s_duration.c_str());  
            
            PlanItem p_item = PlanItem();
            p_item.time = start_time;
            p_item.action = action;
            p_item.duration = duration;
            p_items.push_back(p_item);
        }
        return p_items;
    }
}

/*
    Convert managed plan to Plansys2 msg string, format is the following

    [start_time](action)[duration]\n[start_time](action)[duration]
*/
string ManagedPlan::toPsys2PlanString() const
{
    string result = "";
    vector<PlanItem> pitems = toPsys2Plan().items;
    for(PlanItem item : pitems)
    {
        result += "[" + std::to_string(item.time) + "]";
        result += "(" + item.action + ")";
        result += "[" + std::to_string(item.duration) + "]\n";
    }
    return result;
}

// overload `==` operator 
bool BDIManaged::operator==(ManagedPlan const &mp1, ManagedPlan const &mp2){
     // first check based on target desires
    if(!(mp1.getPlanTarget() == mp2.getPlanTarget()))
        return false;
    if(!(mp1.getFinalTarget() == mp2.getFinalTarget()))
        return false;

    vector<PlanItem> mp1_body = mp1.toPsys2Plan().items;
    vector<PlanItem> mp2_body = mp2.toPsys2Plan().items;

    // check based # of mg. PlanItem in its body 
    if(mp1_body.size() != mp2_body.size())
        return false;

    // check in order mg. PlanItem by  mg. PlanItem
    for(int i=0; i<mp1_body.size(); i++)
        if(mp1_body[i].action != mp2_body[i].action)
            return false;

    //otherwise equals
    return mp1.getPrecondition() == mp2.getPrecondition() && mp1.getContext() == mp2.getContext();
}

// overload `!=` operator 
bool BDIManaged::operator!=(ManagedPlan const &mp1, ManagedPlan const &mp2){
    return !(mp1==mp2);
}

std::ostream& BDIManaged::operator<<(std::ostream& os, const ManagedPlan& mp)
{   
    os << "PLAN\nPlan target: " << mp.getPlanTarget();
    
    os << "\n\nActions exec info:\n";
    for(auto bdi_ai : mp.getActionsExecInfo())
        os << ManagedPlan::computeActionFullName(bdi_ai) << "\n"
            << "Planned Start: " << bdi_ai.planned_start << "\n"
            << "Actual Start: " << bdi_ai.actual_start << "\n"
            << "Exec Time: " << bdi_ai.exec_time << "\n"
            << "Duration: " << bdi_ai.duration << "\n"
            << "Progress: " << bdi_ai.progress << "\n"
            << "STATUS: " << bdi_ai.status << "\n";
    
    os << "\n\nPreconditions to be checked BEFORE execution:\n";
    os << mp.getPrecondition() << "\n";
    
    os << "\n\nContext conditions to be checked DURING execution:\n";
    os << mp.getContext() << "\n";
    
    os << "\n\nPlanned Deadline: " << mp.getPlannedDeadline();
    return os;
}