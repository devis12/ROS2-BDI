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
        ManagedBelief(string name, string type, vector<string> params, float value);
        ManagedBelief(Belief belief);

        string name_;
        string type_;
        vector<string> params_;
        float value_;

        Belief toBelief();
    private:
        

};  // class ManagedBelief

std::ostream& operator<<(std::ostream& os, const ManagedBelief& mb);

// overload `<` operator 
bool operator<(const ManagedBelief& mb1, const ManagedBelief& mb2);

// overload `==` operator 
bool operator==(const ManagedBelief& mb1, const ManagedBelief& mb2);

#endif  // MANAGED_BELIEF_H_