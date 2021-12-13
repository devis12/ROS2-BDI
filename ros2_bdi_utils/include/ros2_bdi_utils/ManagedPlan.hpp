#ifndef MANAGED_PLAN_H_
#define MANAGED_PLAN_H_

#include <vector>
#include <iostream>

#include "plansys2_planner/PlannerClient.hpp"
#include "ros2_bdi_interfaces/msg/bdi_plan.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"

namespace BDIManaged
{

    class ManagedPlan
    {

        public:
            ManagedPlan();
            ManagedPlan(const ManagedDesire& md, const std::vector<plansys2_msgs::msg::PlanItem>& planitems);
            ManagedPlan(const ManagedDesire& md, const std::vector<plansys2_msgs::msg::PlanItem>& planitems, 
                const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context);
            
            ManagedDesire getDesire() const {return desire_;};
            std::vector<plansys2_msgs::msg::PlanItem> getBody() const {return body_;};
            float getPlanDeadline() const {return plan_deadline_;};

            ManagedConditionsDNF getPrecondition() const {return precondition_;};
            ManagedConditionsDNF getContext() const {return context_;};
            
            plansys2_msgs::msg::Plan toPsys2Plan() const;
            ros2_bdi_interfaces::msg::BDIPlan toPlan() const;
        private:

            float computeDeadline(const std::vector<plansys2_msgs::msg::PlanItem>& planitems);

            ManagedDesire desire_;
            std::vector<plansys2_msgs::msg::PlanItem> body_;
            ManagedConditionsDNF precondition_;
            ManagedConditionsDNF context_;
            float plan_deadline_;

    };  // class ManagedPlan

    bool operator==(ManagedPlan const &mp1, ManagedPlan const &mp2);
    std::ostream& operator<<(std::ostream& os, const ManagedPlan& mp);

}

#endif  // MANAGED_PLAN_H_