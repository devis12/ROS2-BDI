#include <yaml-cpp/yaml.h>

#include "ros2_bdi_utils/BDIYAMLParser.hpp"


namespace BDIYAMLParser
{
    /*
        Extract managed beliefs from a YAML file containing them
        throws YAML::InvalidNode, YAML::BadFile, YAML::BadConversion
    */
    vector<ManagedBelief> extractMGBeliefs(const string& bset_filepath)
    {
        vector<ManagedBelief> mgBeliefs;

            
        YAML::Node mybset = YAML::LoadFile(bset_filepath);
        for(YAML::Node::iterator it = mybset.begin(); it != mybset.end(); it++)
        {
            auto yaml_belief = (*it);
            string belief_name = yaml_belief["name"].as<string>();
            int belief_pddl_type = yaml_belief["pddl_type"].as<int>();
            vector<string> belief_params;
            if(yaml_belief["params"].IsDefined())
                for(YAML::Node::iterator it_p = yaml_belief["params"].begin(); it_p != yaml_belief["params"].end(); it_p++)
                    belief_params.push_back((*it_p).as<string>()); 

            float belief_value;
            if(yaml_belief["value"].IsDefined())
                belief_value = yaml_belief["value"].as<float>();

            if(belief_pddl_type == Belief().INSTANCE_TYPE)
                mgBeliefs.push_back(ManagedBelief::buildMBInstance(belief_name, belief_params[0]));
            if(belief_pddl_type == Belief().PREDICATE_TYPE)
                mgBeliefs.push_back(ManagedBelief::buildMBPredicate(belief_name, belief_params));
            if(belief_pddl_type == Belief().FUNCTION_TYPE && yaml_belief["value"].IsDefined())
                mgBeliefs.push_back(ManagedBelief::buildMBFunction(belief_name, belief_params, belief_value));
        }

        return mgBeliefs;
    }

    /*
        Extract managed desires from a YAML file containing them
    */
    vector<ManagedDesire> extractMGDesires(const string& dset_filepath)
    {
        vector<ManagedDesire> mgDesires;
        YAML::Node mydset = YAML::LoadFile(dset_filepath);
        for(YAML::Node::iterator it = mydset.begin(); it != mydset.end(); it++)
        {
            auto yaml_desire = (*it);
            string desire_name = yaml_desire["name"].as<string>();
            float desire_deadline = yaml_desire["deadline"].as<float>();
            if(desire_deadline < 0)
                desire_deadline = 0.0f;
            float desire_priority = yaml_desire["priority"].as<float>();
            if(desire_priority < 0 || desire_priority > 1)
                desire_priority = 0.0f;
            vector<ManagedBelief> desire_value;
            for(YAML::Node::iterator it_val = yaml_desire["value"].begin(); it_val != yaml_desire["value"].end(); it_val++)
            {
                auto yaml_belief = (*it_val);
                string belief_name = yaml_belief["name"].as<string>();
                int belief_pddl_type = yaml_belief["pddl_type"].as<int>();
                vector<string> belief_params;
                if(yaml_belief["params"].IsDefined())
                    for(YAML::Node::iterator it_p = yaml_belief["params"].begin(); it_p != yaml_belief["params"].end(); it_p++)
                        belief_params.push_back((*it_p).as<string>()); 
                
                if(belief_pddl_type == Belief().PREDICATE_TYPE)
                    desire_value.push_back(ManagedBelief::buildMBPredicate(belief_name, belief_params));
            }
                    
            if(desire_value.size() > 0)
                mgDesires.push_back(ManagedDesire{desire_name, desire_value, desire_priority, desire_deadline});
        }       
        return mgDesires;
    }

}// namespace BDIYAMLParser