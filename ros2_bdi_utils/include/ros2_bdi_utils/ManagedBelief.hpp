#ifndef MANAGED_BELIEF_H_
#define MANAGED_BELIEF_H_

#include <string>
#include <vector>
#include <map>
#include <iostream>

#include "ros2_bdi_interfaces/msg/belief.hpp"

const char belief_default_delimiters[2] = {'(', ')'};


/* Namespace for wrapper classes wrt. BDI msgs defined in ros2_bdi_interfaces::msg */
namespace BDIManaged
{
    typedef struct{
        std::string name;
        std::optional<std::vector<std::string>> sub_types;
    }ManagedType;

    typedef struct{
        std::string name;
        ManagedType type;

        bool isPlaceholder(){
            return name.find("{") == 0 && name.find("}") == name.length()-1;
        }
    }ManagedParam;

    /* Wrapper class to easily manage and infer info from a ros2_bdi_interfaces::msg::Belief instance*/
    class ManagedBelief
    {

        public:
            /* Constructor methods */
            ManagedBelief();
            ManagedBelief(const std::string& name,const int& pddl_type,const ManagedType& type);
            ManagedBelief(const std::string& name,const int& pddl_type,const std::vector<ManagedParam>& params, const float& value);
            ManagedBelief(const ros2_bdi_interfaces::msg::Belief& belief);
            
            // Clone a MG Belief DNF
            ManagedBelief clone();

            /*  Static builder methods for a more intuitive managed belief instance constructor methods distinguished
                by the belief type
            */
            static ManagedBelief buildMBInstance(const std::string& name, const std::string& instance_type);
            static ManagedBelief buildMBInstance(const std::string& name, const ManagedType& instance_type);
            static ManagedBelief buildMBPredicate(const std::string& name, const std::vector<ManagedParam>& params);
            static ManagedBelief buildMBFunction(const std::string& name, const std::vector<ManagedParam>& params, const float& value);

            /* getter methods for ManagedBelief instance prop */
            std::string getName() const {return name_;};
            int pddlType() const {return pddl_type_;};
            ManagedType type() const {return type_;};
            std::vector<ManagedParam> getParams() const {return params_;};
            float getValue() const {return value_;};
            std::string pddlTypeString() const;

            /*
                Get param list as a single joined string separated from spaces as per default
            */
            std::string getParamsJoined(const char separator = ' ') const;

            /*  convert instance to ros2_bdi_interfaces::msg::Belief format */
            ros2_bdi_interfaces::msg::Belief toBelief() const;


            /*  convert instance to ros2_bdi_interfaces::msg::Belief format substituting name with respective fulfilling name, which
                indicates we're currently pursuing x (e.g. "f_x" for "x")*/
            ros2_bdi_interfaces::msg::Belief toFulfillmentBelief() const;

            /*
                Try to parse a managed belief from a string, format is the following
                assuming delimiters = ['(', ')']
                ({pddl_type}, {name}, {p1} {p2} {p3} ... {p54}, [{value}])
            */
            static std::optional<ManagedBelief> parseMGBelief(std::string mg_belief, const char delimiters[]);

            /*
                Convert managed belief to string, format is the following
                assuming delimiters = ['(', ')']
                ({pddl_type}, {name}, {p1} {p2} {p3} ... {p54}, [{value}])

                if delimiterNum is wrong, set delimiters to default values ['(',')']
            */
            std::string toString(const char delimiters[]) const;
            std::string toString() const {return toString(belief_default_delimiters);}

            /* substitute placeholders as per assignments map and return a new ManagedBelief instance*/
            ManagedBelief applySubstitution(const std::map<std::string, std::string> assignments) const;
            
        private:
            
            /* name of the belief (instance/predicate/function name) */
            std::string name_;

            /* integer for PDDL TYPE of belief (INSTANCE/PREDICATE/FLUENT)*/
            int pddl_type_; // 1 for INSTANCE ,2 for PREDICATE ,3 for FLUENT/FUNCTION, check ros2_bdi_interfaces::msg::Belief

            // actually valuable just for instances
            ManagedType type_;

            /* vector of parameters for PREDICATE(2)/FLUENT(3) belief type, single value representing instance type for INSTANCE(1) belief type*/
            std::vector<ManagedParam> params_;

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