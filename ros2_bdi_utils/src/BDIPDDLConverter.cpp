#include "ros2_bdi_utils/BDIPDDLConverter.hpp"

#include "ros2_bdi_interfaces/msg/belief.hpp"

using std::string;
using ros2_bdi_interfaces::msg::Belief;

namespace BDIPDDLConverter
{

    /*
      Build plansys2::Instance obj from ManagedBelief
    */
    plansys2::Instance buildInstance(const BDIManaged::ManagedBelief& mb)
    {
        return plansys2::Instance{mb.getName(), mb.type().name};
    }

    /*
        Build plansys2::Predicate obj from ManagedBelief
    */
    plansys2::Predicate buildPredicate(const BDIManaged::ManagedBelief& mb)
    {
        return plansys2::Predicate{"(" + mb.getName()+ " " + mb.getParamsJoined() + ")"};
    }

    /*
        Build plansys2::Function obj from ManagedBelief
    */
    plansys2::Function buildFunction(const BDIManaged::ManagedBelief& mb)
    {
        return plansys2::Function{"(" + mb.getName()+ " " + mb.getParamsJoined() + " " + std::to_string(mb.getValue()) + ")"};;
    }

    /*
        Convert Desire into PDDL Goal
    */
    std::string desireToGoal(const ros2_bdi_interfaces::msg::Desire& desire)
    {
        string goal_string = "(and ";
        
        for(Belief b : desire.value)
        {
        if(b.pddl_type == Belief().PREDICATE_TYPE){
            string params_list = "";
            for(int i=0; i<b.params.size(); i++)
                params_list += (i==b.params.size()-1)? b.params[i] : b.params[i] + " ";
            
            goal_string += "(" + b.name + " " + params_list + ")";
        }
            
        }
        
        goal_string += ")";

        return goal_string;
    }
}
