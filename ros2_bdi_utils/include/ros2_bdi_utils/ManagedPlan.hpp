#ifndef MANAGED_PLAN_H_
#define MANAGED_PLAN_H_

#include <string>
#include <vector>
#include <iostream>

#include "plansys2_planner/PlannerClient.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"

#include "ros2_bdi_interfaces/msg/bdi_plan.hpp"

using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

using ros2_bdi_interfaces::msg::BDIPlan;

using std::string;
using std::vector;

class ManagedPlan
{

    public:
        ManagedPlan();
        ManagedPlan(const ManagedDesire& md, const vector<PlanItem>& planitems);

        ManagedDesire desire_;
        vector<PlanItem> body_;
        vector<ManagedBelief> precondition_;
        vector<ManagedBelief> context_;
        float plan_deadline_;

        Plan toPsys2Plan() const;
        BDIPlan toPlan() const;
    private:
        vector<ManagedBelief> computePrecondition(const vector<PlanItem>& planitems);
        vector<ManagedBelief> computeContext(const vector<PlanItem>& planitems);

        float computeDeadline(const vector<PlanItem>& planitems);
        

};  // class ManagedPlan

bool operator==(ManagedPlan const &mp1, ManagedPlan const &mp2);
std::ostream& operator<<(std::ostream& os, const ManagedPlan& mp);

#endif  // MANAGED_PLAN_H_