#ifndef MANAGED_PLAN_H_
#define MANAGED_PLAN_H_

#include <vector>
#include <iostream>

#include "plansys2_planner/PlannerClient.hpp"
#include "ros2_bdi_interfaces/msg/bdi_plan.hpp"
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
            std::vector<plansys2_msgs::msg::PlanItem> getBody() const {return body_;};
            float getPlanDeadline() const {return plan_deadline_;};

            ManagedConditionsDNF getPrecondition() const {return precondition_;};
            ManagedConditionsDNF getContext() const {return context_;};
            
            /* convert instance to plansys2::msg::Plan format */
            plansys2_msgs::msg::Plan toPsys2Plan() const;

            /* convert instance to ros2_bdi_interfaces::msg::BDIPlan format*/
            ros2_bdi_interfaces::msg::BDIPlan toPlan() const;
        private:

            /* Compute deadline estimate based on current actions estimated duration within the one listed in the plan */
            float computeDeadline(const std::vector<plansys2_msgs::msg::PlanItem>& planitems);

            /* Desire to be fulfilled by plan execution*/
            ManagedDesire target_;

            /* Plansys2 action (name, duration, start time) vector enwrapping the tree of actions to be performed to fulfilled the desire
                that needs to be passed to PlanSys2 Executor */
            std::vector<plansys2_msgs::msg::PlanItem> body_;

            /* Condition clauses in a DNF expression that must be verified before plan exec starts */
            ManagedConditionsDNF precondition_;

            /* Condition clauses in a DNF expression that must be verified over all plan exec */
            ManagedConditionsDNF context_;

            /* Estimated plan deadline */
            float plan_deadline_;

    };  // class ManagedPlan

    bool operator==(ManagedPlan const &mp1, ManagedPlan const &mp2);
    std::ostream& operator<<(std::ostream& os, const ManagedPlan& mp);

}

#endif  // MANAGED_PLAN_H_