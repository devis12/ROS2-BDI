#ifndef MANAGED_DESIRE_H_
#define MANAGED_DESIRE_H_

#include <string>
#include <vector>
#include <iostream>

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"

#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_interfaces/msg/condition.hpp"

using std::string;
using std::vector;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::DesireSet;
using ros2_bdi_interfaces::msg::Condition;

class ManagedDesire
{

    public:
        ManagedDesire();
        ManagedDesire(const string& name,const vector<ManagedBelief>& value,const float& priority,const float& deadline);
        ManagedDesire(const string& name,const vector<ManagedBelief>& value,const float& priority,const float& deadline,
                        const vector<ManagedCondition>& precondition, const vector<ManagedCondition>& context);
        ManagedDesire(const Desire& desire);

        string getName() const {return name_;};
        vector<ManagedBelief> getValue() const {return value_;};
        float getPriority() const {return priority_;}
        float getDeadline() const {return deadline_;}
        vector<ManagedCondition> getPrecondition() const {return precondition_;}
        vector<ManagedCondition> getContext() const {return context_;}

        Desire toDesire() const;
    private:
        string name_;
        vector<ManagedBelief> value_;
        float priority_;
        float deadline_;
        vector<ManagedCondition> precondition_;
        vector<ManagedCondition> context_;

};  // class ManagedDesire

std::ostream& operator<<(std::ostream& os, const ManagedDesire& md);

// overload `<` operator 
bool operator<(const ManagedDesire& md1, const ManagedDesire& md2);

// overload `==` operator 
bool operator==(const ManagedDesire& md1, const ManagedDesire& md2);

#endif  // MANAGED_DESIRE_H_