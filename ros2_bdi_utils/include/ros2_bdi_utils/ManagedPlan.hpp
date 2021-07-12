#ifndef MANAGED_PLAN_H_
#define MANAGED_PLAN_H_

#include <string>
#include <vector>
#include <iostream>

#include "plansys2_planner/PlannerClient.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"

using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

using std::string;
using std::vector;

class ManagedPlan
{

    public:
        ManagedPlan();
        ManagedPlan(const ManagedDesire& md, const Plan& plan);

        ManagedDesire desire_;
        vector<PlanItem> body_;
        vector<ManagedBelief> precondition_;
        vector<ManagedBelief> context_;
        float plan_deadline_;

        Plan toPlan() const;
    private:
        vector<Belief> computePrecondition(const Plan& plan);
        vector<Belief> computeContext(const Plan& plan);

        float computeDeadline(const Plan& plan);
        

};  // class ManagedPlan

std::ostream& operator<<(std::ostream& os, const ManagedPlan& mp);

#endif  // MANAGED_PLAN_H_