#ifndef MANAGED_PLAN_H_
#define MANAGED_PLAN_H_

#include <string>
#include <vector>
#include <iostream>

#include "plansys2_planner/PlannerClient.hpp"
#include "ros2_bdi_interfaces/msg/bdi_plan.hpp"
#include "ros2_bdi_interfaces/msg/bdi_action_execution_info.hpp"
#include "ros2_bdi_interfaces/msg/bdi_plan_execution_info.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"


/* Namespace for wrapper classes wrt. BDI msgs defined in ros2_bdi_interfaces::msg */
namespace BDIManaged
{

    /* Wrapper class to easily manage and infer info from a ros2_bdi_interfaces::msg::Plan instance*/
    class ManagedPlan
    {

        public:
            /* Constructor methods */
            ManagedPlan();
            ManagedPlan(const ManagedDesire& md, const std::vector<plansys2_msgs::msg::PlanItem>& planitems);
            ManagedPlan(const ManagedDesire& md, const std::vector<plansys2_msgs::msg::PlanItem>& planitems, 
                const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context);
            
            /* getter methods for ManagedPlan instance prop  */
            ManagedDesire getDesire() const {return target_;};

            void setUpdatedInfo(const ros2_bdi_interfaces::msg::BDIPlanExecutionInfo& planExecInfo)
            {
                this->exec_status_ = planExecInfo.status;
                this->last_current_time_ = planExecInfo.current_time;
                std::vector<ros2_bdi_interfaces::msg::BDIActionExecutionInfo> actions_exec_info = 
                    std::vector<ros2_bdi_interfaces::msg::BDIActionExecutionInfo> (planExecInfo.actions_exec_info.begin(), planExecInfo.actions_exec_info.end());
                this->actions_exec_info_ = actions_exec_info;
            }

            std::vector<ros2_bdi_interfaces::msg::BDIActionExecutionInfo> getActionsExecInfo() const {return actions_exec_info_;};
            float getPlannedDeadline() const {return planned_deadline_;};
            
            /*Already started/executing actions start/end time taken in consideration for estimating the new deadline at run time*/
            //TODO fix the logics!!!
            float getUpdatedEstimatedDeadline();

            ManagedConditionsDNF getPrecondition() const {return precondition_;};
            ManagedConditionsDNF getContext() const {return context_;};
            
            /* convert instance to plansys2::msg::Plan format */
            plansys2_msgs::msg::Plan toPsys2Plan() const;

            /* convert instance to ros2_bdi_interfaces::msg::BDIPlan format*/
            ros2_bdi_interfaces::msg::BDIPlan toPlan() const;

            /*
                Example 
                From action name "a1" and args ["p1", "p2"]
                return "(a1 p1 p2)"
            */
            static std::string computeActionFullName(ros2_bdi_interfaces::msg::BDIActionExecutionInfo action_exec);

        private:

            /*
                compute actoin exec info from plan item, execution status UNKNOWN
            */
            static std::vector<ros2_bdi_interfaces::msg::BDIActionExecutionInfo> 
                computeActionsExecInfo(std::vector<plansys2_msgs::msg::PlanItem> plan_items);

            /* Compute deadline estimate based on current actions estimated duration within the one listed in the plan */
            float computePlannedDeadline();



            float computeUpdatedEndTime(const ros2_bdi_interfaces::msg::BDIActionExecutionInfo& bdi_ai);

            /* Desire to be fulfilled by plan execution*/
            ManagedDesire target_;

            /* Plansys2 action (name, duration, start time) vector enwrapping the tree of actions to be performed to fulfilled the desire
                that needs to be passed to PlanSys2 Executor */
            std::vector<ros2_bdi_interfaces::msg::BDIActionExecutionInfo> actions_exec_info_;

            /* Condition clauses in a DNF expression that must be verified before plan exec starts */
            ManagedConditionsDNF precondition_;

            /* Condition clauses in a DNF expression that must be verified over all plan exec */
            ManagedConditionsDNF context_;

            /* Planned deadline */
            float planned_deadline_;

            /* Last reported current time */
            float last_current_time_;

            /* Plan level exec status*/
            std::string exec_status_;

            

    };  // class ManagedPlan

    bool operator==(ManagedPlan const &mp1, ManagedPlan const &mp2);
    bool operator!=(ManagedPlan const &mp1, ManagedPlan const &mp2);
    std::ostream& operator<<(std::ostream& os, const ManagedPlan& mp);

}

#endif  // MANAGED_PLAN_H_