#ifndef BDIYAMLParser__UTILS_H_
#define BDIYAMLParser__UTILS_H_

#include <string>
#include <vector>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"

using std::string;
using std::vector;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::DesireSet;

namespace BDIYAMLParser
{

    /*
        Extract managed beliefs from a YAML file containing them
        throws YAML::InvalidNode, YAML::BadFile, YAML::BadConversion
    */
    vector<ManagedBelief> extractMGBeliefs(const string& bset_filepath);

    /*
        Extract managed desires from a YAML file containing them
    */
    vector<ManagedDesire> extractMGDesires(const string& dset_filepath);

}  // namespace BDIYAMLParser

#endif  // BDIYAMLParser__UTILS_H_