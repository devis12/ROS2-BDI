#ifndef MANAGED_DESIRE_H_
#define MANAGED_DESIRE_H_

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <iostream>

#include "ros2_bdi_interfaces/msg/desire.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedCondition.hpp"
#include "ros2_bdi_utils/ManagedConditionsConjunction.hpp"
#include "ros2_bdi_utils/ManagedConditionsDNF.hpp"

namespace BDIManaged
{

    class ManagedDesire
    {

        public:
            ManagedDesire();
            
            ManagedDesire(const std::string& name,const std::vector<ManagedBelief>& value,const float& priority,const float& deadline);
            
            ManagedDesire(const std::string& name, const std::string& desire_group, const std::vector<ManagedBelief>& value,
                const float& priority,const float& deadline);

            ManagedDesire(const std::string& name,const std::vector<ManagedBelief>& value,const float& priority,const float& deadline,
                            const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context);
            
            ManagedDesire(const std::string& name,const std::vector<ManagedBelief>& value,const float& priority,const float& deadline,
                            const std::vector<ManagedBelief>& rollbackBeliefsAdd, const std::vector<ManagedBelief>& rollbackBeliefsDel);
            
            ManagedDesire(const std::string& name,const std::vector<ManagedBelief>& value,const float& priority,const float& deadline,
                            const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context,
                            const std::vector<ManagedBelief>& rollbackBeliefsAdd, const std::vector<ManagedBelief>& rollbackBeliefsDel);
            
            ManagedDesire(const ros2_bdi_interfaces::msg::Desire& desire);

            std::string getName() const {return name_;};
            std::vector<ManagedBelief> getValue() const {return value_;};
            float getPriority() const {return priority_;}
            float getDeadline() const {return deadline_;}
            void setDesireGroup(const std::string& desire_group){desire_group_ = desire_group;}
            std::string getDesireGroup() const {return desire_group_;}
            ManagedConditionsDNF getPrecondition() const {return precondition_;}
            ManagedConditionsDNF getContext() const {return context_;}
            std::vector<ManagedBelief> getRollbackBeliefAdd() const {return rollback_belief_add_;}
            std::vector<ManagedBelief> getRollbackBeliefDel() const {return rollback_belief_del_;}
            
            bool hasParent() const { return parent_ != NULL;}
            void setParent(const ManagedDesire& parent) {parent_ = std::make_shared<ManagedDesire>(parent);}
            ManagedDesire getParent() {
                if(hasParent())
                    return *parent_;
                else
                    return ManagedDesire{};
            };

            ros2_bdi_interfaces::msg::Desire toDesire() const;
            // return true if empty target or if target appears to be achieved in the passed bset
            bool isFulfilled(const std::set<ManagedBelief>& bset);
        private:
            std::string name_;
            std::string desire_group_;
            std::vector<ManagedBelief> value_;
            std::shared_ptr<ManagedDesire> parent_;
            float priority_;
            float deadline_;
            ManagedConditionsDNF precondition_;
            ManagedConditionsDNF context_;
            std::vector<ManagedBelief> rollback_belief_add_;
            std::vector<ManagedBelief> rollback_belief_del_;

    };  // class ManagedDesire

    std::ostream& operator<<(std::ostream& os, const ManagedDesire& md);

    // overload `<` operator 
    bool operator<(const ManagedDesire& md1, const ManagedDesire& md2);

    // overload `==` operator 
    bool operator==(const ManagedDesire& md1, const ManagedDesire& md2);

}

#endif  // MANAGED_DESIRE_H_