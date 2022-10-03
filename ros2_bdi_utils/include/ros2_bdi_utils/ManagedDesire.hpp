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

/* Namespace for wrapper classes wrt. BDI msgs defined in ros2_bdi_interfaces::msg */
namespace BDIManaged
{

    /* Wrapper class to easily manage and infer info from a ros2_bdi_interfaces::msg::Desire instance*/
    class ManagedDesire
    {

        public:
            /* Constructor methods*/
            ManagedDesire();
            
            ManagedDesire(const std::string& name,const std::vector<ManagedBelief>& value,const float& priority,const float& deadline);
            
            ManagedDesire(const std::string& name,const std::vector<ManagedBelief>& value,const float& priority,const float& deadline,
                            const ManagedConditionsDNF& precondition, const ManagedConditionsDNF& context,
                            const std::vector<ManagedBelief>& rollbackBeliefsAdd, const std::vector<ManagedBelief>& rollbackBeliefsDel);
            
            ManagedDesire(const ros2_bdi_interfaces::msg::Desire& desire);

            // Clone a MG Desire
            ManagedDesire clone();

            /* Getter/setter methods for ManagedDesire instance properties*/
            
            void setName(const std::string& name){name_ = std::string{name};};
            std::string getName() const {return name_;};
            std::string getNameValue() const {
                std::string dvalue = "";
                for(auto bv : value_)
                    dvalue += "(" + bv.getName() + " " + bv.getParamsJoined() + ")";
                return name_ + ": " + dvalue;
            };
            std::vector<ManagedBelief> getValue() const {return value_;};
            float getPriority() const {return priority_;}
            float getDeadline() const {return deadline_;}
            void setDesireGroup(const std::string& desire_group){desire_group_ = desire_group;}
            std::string getDesireGroup() const {return desire_group_;}
            ManagedConditionsDNF getPrecondition() const {return precondition_;}
            ManagedConditionsDNF getContext() const {return context_;}
            std::vector<ManagedBelief> getRollbackBeliefAdd() const {return rollback_belief_add_;}
            std::vector<ManagedBelief> getRollbackBeliefDel() const {return rollback_belief_del_;}
            
            /*  Returns true if parent exists, otherwise false*/
            bool hasParent() const { return parent_ != NULL;}
            void setParent(const ManagedDesire& parent) {parent_ = std::make_shared<ManagedDesire>(parent);}

            /*  Returns parent if exists, otherwise empty ManagedDesire*/
            ManagedDesire getParent() {
                if(hasParent())
                    return *parent_;
                else
                    return ManagedDesire{};
            };
            
            /* Convert to raw Desire msg as per ros2_bdi_interfaces::msg::Desire */
            ros2_bdi_interfaces::msg::Desire toDesire() const;

            // return true if empty target or if target appears to be achieved in the passed bset
            bool isFulfilled(const std::set<ManagedBelief>& bset);
            
            // return true if otherDesire presents the same exact target value, regardless of other attributes (preconditions, context, deadline,...)
            bool equivalentValue(const ManagedDesire& otherDesire);

            // return true if otherDesire presents the same exact name, priority and desire group, belief set should be a subset of the belief set of otherDesire
            bool equalsOrSupersetIgnoreAdvancedInfo(const ManagedDesire& otherDesire);
            
            /* Base boosting conditions (name,priority, desire group) checked wrt otherDesire*/
            bool baseBoostingConditionsMatch(const BDIManaged::ManagedDesire& otherDesire);

            /* Given another desire check whether is matching for boosting:
                -NO:  return empty array
                -YES: return array with additional boosting value
            */
            std::vector<BDIManaged::ManagedBelief> computeBoostingValue(const BDIManaged::ManagedDesire& otherDesire);

            // return true if otherDesire is augmented to the current one
            bool boostDesire(const ManagedDesire& otherDesire);

            /* substitute placeholders as per assignments map and return a new ManagedDesire instance*/
            ManagedDesire applySubstitution(const std::map<std::string, std::string> assignments) const;
        private:
            /*name of the desire*/
            std::string name_;

            /* desire group id that the desire might belong to (precondition/context condition, look parent_)*/
            std::string desire_group_;
            
            /* belief to be made true by fulfilling the desire*/
            std::vector<ManagedBelief> value_;
            
            // desire might have a parent, if it's a precursor to the parent 
            // (desire to fulfill preconditions or context condition which belong to the same group
            // have all the same parent which is the original desire from which condition is extracted)
            std::shared_ptr<ManagedDesire> parent_;

            /* priority of the desire in the [0-1 range]*/
            float priority_;

            /* desired deadline for the desire fulfillment (currently considered just wrt. single plan exec.) */
            float deadline_;

            /* conditions that need to be made true in order to start a plan exec for given desire fulfillment */
            ManagedConditionsDNF precondition_;

            /* conditions that need to be true along all plan exec for given desire fulfillment */
            ManagedConditionsDNF context_;

            /*  beliefs that needs to be added to the belief set in case of plan exec. abortion 
                during plan exec for the given desire fulfillment
            */
            std::vector<ManagedBelief> rollback_belief_add_;

            /*  beliefs that needs to be deleted from the belief set in case of plan exec. abortion 
                during plan exec for the given desire fulfillment
            */
            std::vector<ManagedBelief> rollback_belief_del_;

    };  // class ManagedDesire

    std::ostream& operator<<(std::ostream& os, const ManagedDesire& md);

    // overload `<` operator 
    bool operator<(const ManagedDesire& md1, const ManagedDesire& md2);

    // overload `==` operator 
    bool operator==(const ManagedDesire& md1, const ManagedDesire& md2);

    // overload `!=` operator 
    bool operator!=(const ManagedDesire& md1, const ManagedDesire& md2);

}

#endif  // MANAGED_DESIRE_H_