#ifndef BDIYAMLParser__UTILS_H_
#define BDIYAMLParser__UTILS_H_

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

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

    /*
        Given a YAML Node which should represent a desire, parse it and build a ManagedDesire
        return std::nullopt if not possible
    */
    std::optional<ManagedDesire> parseMGDesire(YAML::Node& yaml_desire);

    /*
        Given a YAML Node which should represent a belief, parse it and build a ManagedBelief
        return std::nullopt if not possible
    */
    std::optional<ManagedBelief> parseMGBelief(YAML::Node& yaml_belief);

    /*
        Given a YAML Node which should represent an array of beliefs, parse it and build a vector<ManagedBelief>
        return empty if there isn't any belief available within the node
    */
    vector<ManagedBelief> parseMGBeliefs(YAML::Node& yaml_beliefs);

     /*
        Given a YAML Node which should represent an array of beliefs, parse it and build a vector<ManagedBelief>
        containing just the beliefs of the given type (if ALL_TYPE, do not filter, returns all beliefs of any given/valid type)
        return empty if there isn't any belief available within the node
    */
    vector<ManagedBelief> parseMGBeliefs(YAML::Node& yaml_beliefs, const int& belief_type);

    /*
        Given a YAML node which should represent a YAML belief, retrieve its parameters (if any)
    */
    vector<string> parseBeliefParams(YAML::Node& yaml_belief);

    /*
        Given a YAML node representing a desire and the name of the condition vector (e.g. "precondition", "context"),
        extract a DNF clause of managed condition
    */
    ManagedConditionsDNF parseMGConditionsDNF(YAML::Node& yaml_desire, const string& condition_vect_name);

}  // namespace BDIYAMLParser

#endif  // BDIYAMLParser__UTILS_H_