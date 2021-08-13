#ifndef MANAGED_DESIRE_H_
#define MANAGED_DESIRE_H_

#include <string>
#include <vector>
#include <memory>
#include <iostream>

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedConditionsConjunction.hpp"
#include "ros2_bdi_utils/ManagedConditionsDNF.hpp"

#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_interfaces/msg/condition.hpp"
#include "ros2_bdi_interfaces/msg/conditions_conjunction.hpp"
#include "ros2_bdi_interfaces/msg/conditions_dnf.hpp"

using std::string;
using std::vector;

using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::DesireSet;
using ros2_bdi_interfaces::msg::Condition;
using ros2_bdi_interfaces::msg::ConditionsConjunction;
using ros2_bdi_interfaces::msg::ConditionsDNF;

class ManagedDesire
{

    public:
        ManagedDesire();
        
        ManagedDesire(const string& name,const vector<ManagedBelief>& value,const float& priority,const float& deadline);
        
        ManagedDesire(const string& name, const string& desire_group, const vector<ManagedBelief>& value,
            const float& priority,const float& deadline);

        ManagedDesire(const string& name,const vector<ManagedBelief>& value,const float& priority,const float& deadline,
                        const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context);
        
        ManagedDesire(const Desire& desire);

        string getName() const {return name_;};
        vector<ManagedBelief> getValue() const {return value_;};
        float getPriority() const {return priority_;}
        float getDeadline() const {return deadline_;}
        void setDesireGroup(const string& desire_group){desire_group_ = desire_group;}
        string getDesireGroup() const {return desire_group_;}
        ManagedConditionsDNF getPrecondition() const {return precondition_;}
        ManagedConditionsDNF getContext() const {return context_;}
        
        bool hasParent() const { return parent_ != NULL;}
        void setParent(const ManagedDesire& parent) {parent_ = std::make_shared<ManagedDesire>(parent);}
        ManagedDesire getParent() {
            if(hasParent())
                return *parent_;
            else
                return ManagedDesire{};
        };

        Desire toDesire() const;
        // return true if empty target or if target appears to be achieved in the passed bset
        bool isFulfilled(const set<ManagedBelief>& bset);
    private:
        string name_;
        string desire_group_;
        vector<ManagedBelief> value_;
        std::shared_ptr<ManagedDesire> parent_;
        float priority_;
        float deadline_;
        ManagedConditionsDNF precondition_;
        ManagedConditionsDNF context_;

};  // class ManagedDesire

std::ostream& operator<<(std::ostream& os, const ManagedDesire& md);

// overload `<` operator 
bool operator<(const ManagedDesire& md1, const ManagedDesire& md2);

// overload `==` operator 
bool operator==(const ManagedDesire& md1, const ManagedDesire& md2);

#endif  // MANAGED_DESIRE_H_