#ifndef MANAGED_BELIEF_H_
#define MANAGED_BELIEF_H_

#include <string>
#include <vector>
#include <iostream>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"

using std::string;
using std::vector;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;

class ManagedBelief
{

    public:
        ManagedBelief(const Belief& belief);
        
        static ManagedBelief buildMBInstance(const string& name, const string& instance_type);
        static ManagedBelief buildMBPredicate(const string& name, const vector<string>& params);
        static ManagedBelief buildMBFunction(const string& name, const vector<string>& params, const float& value);

        string getName() const {return name_;};
        int pddlType() const {return pddl_type_;};
        vector<string> getParams() const {return params_;};
        float getValue() const {return value_;};
        string pddlTypeString() const;

        Belief toBelief() const;
        
    private:
        ManagedBelief(const string& name,const int& type,const vector<string>& params,const float& value);
        
        string name_;
        int pddl_type_;
        vector<string> params_;
        float value_;

};  // class ManagedBelief

std::ostream& operator<<(std::ostream& os, const ManagedBelief& mb);

// overload `<` operator 
bool operator<(const ManagedBelief& mb1, const ManagedBelief& mb2);

// overload `==` operator 
bool operator==(const ManagedBelief& mb1, const ManagedBelief& mb2);

#endif  // MANAGED_BELIEF_H_