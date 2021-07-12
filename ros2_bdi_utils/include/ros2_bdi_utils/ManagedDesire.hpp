#ifndef MANAGED_DESIRE_H_
#define MANAGED_DESIRE_H_

#include <string>
#include <vector>
#include <iostream>

#include "ros2_bdi_utils/ManagedBelief.hpp"

#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"

using std::string;
using std::vector;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::DesireSet;

class ManagedDesire
{

    public:
        ManagedDesire();
        ManagedDesire(const string& name,const vector<ManagedBelief>& value,const float& priority,const float& deadline);
        ManagedDesire(const Desire& desire);

        string name_;
        vector<ManagedBelief> value_;
        float priority_;
        float deadline_;

        Desire toDesire() const;
    private:
        

};  // class ManagedDesire

std::ostream& operator<<(std::ostream& os, const ManagedDesire& md);

// overload `<` operator 
bool operator<(const ManagedDesire& md1, const ManagedDesire& md2);

// overload `==` operator 
bool operator==(const ManagedDesire& md1, const ManagedDesire& md2);

#endif  // MANAGED_DESIRE_H_