#ifndef MANAGED_BELIEF_H_
#define MANAGED_BELIEF_H_

#include <string>
#include <vector>
#include <iostream>

#include "ros2_bdi_interfaces/msg/belief.hpp"

/* Namespace for wrapper classes wrt. BDI msgs defined in ros2_bdi_interfaces::msg */
namespace BDIManaged
{

    /* Wrapper class to easily manage and infer info from a ros2_bdi_interfaces::msg::Belief instance*/
    class ManagedBelief
    {

        public:
            /* Constructor methods */
            ManagedBelief();
            ManagedBelief(const std::string& name,const int& type,const std::vector<std::string>& params,const float& value);
            ManagedBelief(const ros2_bdi_interfaces::msg::Belief& belief);
            
            /*  Static builder methods for a more intuitive managed belief instance constructor methods distinguished
                by the belief type
            */
            static ManagedBelief buildMBInstance(const std::string& name, const std::string& instance_type);
            static ManagedBelief buildMBPredicate(const std::string& name, const std::vector<std::string>& params);
            static ManagedBelief buildMBFunction(const std::string& name, const std::vector<std::string>& params, const float& value);

            /* getter methods for ManagedBelief instance prop */
            std::string getName() const {return name_;};
            int pddlType() const {return pddl_type_;};
            std::vector<std::string> getParams() const {return params_;};
            float getValue() const {return value_;};
            std::string pddlTypeString() const;

            /*
                Get param list as a single joined string separated from spaces as per default
            */
            std::string getParamsJoined(const char separator = ' ') const;

            /*  convert instance to ros2_bdi_interfaces::msg::Belief format */
            ros2_bdi_interfaces::msg::Belief toBelief() const;
            
        private:
            
            /* name of the belief (instance/predicate/function name) */
            std::string name_;

            /* integer for PDDL TYPE of belief (INSTANCE/PREDICATE/FLUENT)*/
            int pddl_type_; // 1 for INSTANCE ,2 for PREDICATE ,3 for FLUENT/FUNCTION, check ros2_bdi_interfaces::msg::Belief

            /* vector of parameters for PREDICATE(2)/FLUENT(3) belief type, single value representing instance type for INSTANCE(1) belief type*/
            std::vector<std::string> params_;

            /* value used in case of FLUENT belief type*/
            float value_;

    };  // class ManagedBelief

    std::ostream& operator<<(std::ostream& os, const ManagedBelief& mb);

    // overload `<` operator 
    bool operator<(const ManagedBelief& mb1, const ManagedBelief& mb2);

    // overload `==` operator 
    bool operator==(const ManagedBelief& mb1, const ManagedBelief& mb2);
}


#endif  // MANAGED_BELIEF_H_