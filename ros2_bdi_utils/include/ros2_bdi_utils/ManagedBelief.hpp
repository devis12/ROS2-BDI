#ifndef MANAGED_BELIEF_H_
#define MANAGED_BELIEF_H_

#include <string>
#include <vector>
#include <iostream>

#include "ros2_bdi_interfaces/msg/belief.hpp"

namespace BDIManaged
{

    class ManagedBelief
    {

        public:
            ManagedBelief();
            ManagedBelief(const ros2_bdi_interfaces::msg::Belief& belief);
            
            static ManagedBelief buildMBInstance(const std::string& name, const std::string& instance_type);
            static ManagedBelief buildMBPredicate(const std::string& name, const std::vector<std::string>& params);
            static ManagedBelief buildMBFunction(const std::string& name, const std::vector<std::string>& params, const float& value);

            std::string getName() const {return name_;};
            int pddlType() const {return pddl_type_;};
            std::vector<std::string> getParams() const {return params_;};
            float getValue() const {return value_;};
            std::string pddlTypeString() const;

            ros2_bdi_interfaces::msg::Belief toBelief() const;
            
        private:
            ManagedBelief(const std::string& name,const int& type,const std::vector<std::string>& params,const float& value);
            
            std::string name_;
            int pddl_type_;
            std::vector<std::string> params_;
            float value_;

    };  // class ManagedBelief

    std::ostream& operator<<(std::ostream& os, const ManagedBelief& mb);

    // overload `<` operator 
    bool operator<(const ManagedBelief& mb1, const ManagedBelief& mb2);

    // overload `==` operator 
    bool operator==(const ManagedBelief& mb1, const ManagedBelief& mb2);
}


#endif  // MANAGED_BELIEF_H_