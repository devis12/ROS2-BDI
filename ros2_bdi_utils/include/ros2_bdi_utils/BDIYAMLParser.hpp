#ifndef BDIYAMLParser__UTILS_H_
#define BDIYAMLParser__UTILS_H_

#include <optional>
#include <string>
#include <vector>
#include <set>

#include "plansys2_msgs/msg/param.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp" 

#include <yaml-cpp/yaml.h>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_utils/ManagedReactiveRule.hpp"

namespace BDIYAMLParser
{

    /*
        Extract managed beliefs from a YAML file containing them
        throws YAML::InvalidNode, YAML::BadFile, YAML::BadConversion
    */
    std::vector<BDIManaged::ManagedBelief> extractMGBeliefs(const std::string& bset_filepath, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert);

    /*
        Extract managed desires from a YAML file containing them
    */
    std::vector<BDIManaged::ManagedDesire> extractMGDesires(const std::string& dset_filepath, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert);

    /*
        Extract managed reactive rules from a YAML file containing them
    */
    std::set<BDIManaged::ManagedReactiveRule> extractMGReactiveRules(const std::string& mgrrules_filepath, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert);  

    /*
        Parse and get managed reactive rules from a YAML node containing them
    */
    std::set<BDIManaged::ManagedReactiveRule> parseMGReactiveRules(YAML::Node& yaml_rrules, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert);  

    /*
        Given a YAML Node which should represent a desire, parse it and build a ManagedDesire
        return std::nullopt if not possible
    */
    std::optional<BDIManaged::ManagedDesire> parseMGDesire(YAML::Node& yaml_desire, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert);

    /*
        Given a YAML Node which should represent a belief, parse it and build a ManagedBelief
        return std::nullopt if not possible
    */
    std::optional<BDIManaged::ManagedBelief> parseMGBelief(YAML::Node& yaml_belief, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert);

    /*
        Given a YAML Node which should represent an array of beliefs, parse it and build a vector<ManagedBelief>
        return empty if there isn't any belief available within the node
    */
    std::vector<BDIManaged::ManagedBelief> parseMGBeliefs(YAML::Node& yaml_beliefs, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert);

     /*
        Given a YAML Node which should represent an array of beliefs, parse it and build a vector<ManagedBelief>
        containing just the beliefs of the given type (if ALL_TYPE, do not filter, returns all beliefs of any given/valid type)
        return empty if there isn't any belief available within the node
    */
    std::vector<BDIManaged::ManagedBelief> parseMGBeliefs(YAML::Node& yaml_beliefs, const int& belief_type, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert);

    /*
        Given a YAML node which should represent a YAML belief, retrieve its parameters (if any)
    */
    std::vector<BDIManaged::ManagedParam> parseBeliefParams(YAML::Node& yaml_belief, const std::vector<plansys2_msgs::msg::Param>& params_def);

    /*
        Given a YAML parent node containing a condition expressed in DNF and the name of the condition vector (e.g. "precondition", "context"),
        extract a vector of managed condition DNF clause
    */
    BDIManaged::ManagedConditionsDNF parseMGConditionsDNF(YAML::Node& yaml_parent_node, const std::string& condition_vect_name, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert);

}  // namespace BDIYAMLParser

#endif  // BDIYAMLParser__UTILS_H_