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
            std::optional<ManagedBelief> opt_mb = parseMGBelief(yaml_belief);

            if(opt_mb.has_value())
                mgBeliefs.push_back(opt_mb.value());
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
            std::optional<ManagedDesire> opt_md = parseMGDesire(yaml_desire);
                    
            if(opt_md.has_value())
                mgDesires.push_back(opt_md.value());
        }       
        return mgDesires;
    }

    /*
        Given a YAML Node which should represent a desire, parse it and build a ManagedDesire
        return std::nullopt if not possible
    */
    std::optional<ManagedDesire> parseMGDesire(YAML::Node& yaml_desire)
    {
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
            std::optional<ManagedBelief> opt_mb = parseMGBelief(yaml_belief);
            
            if(opt_mb.has_value() && opt_mb.value().pddlType() == Belief().PREDICATE_TYPE)
                desire_value.push_back(opt_mb.value());
        }

        // retrieving preconditions and context (if there is any, otherwise empty)
        vector<ManagedCondition> preconditions = retrieveMGConditions(yaml_desire, "precondition");
        vector<ManagedCondition> context = retrieveMGConditions(yaml_desire, "context");
                
        if(desire_value.size() > 0)
            return(ManagedDesire{desire_name, desire_value, 
                desire_priority, desire_deadline, preconditions, context});
        else
            return std::nullopt;
    }

    /*
        Given a YAML Node which should represent a belief, parse it and build a ManagedBelief
        return std::nullopt if not possible
    */
    std::optional<ManagedBelief> parseMGBelief(YAML::Node& yaml_belief)
    {
        string belief_name = yaml_belief["name"].as<string>();
        int belief_pddl_type = yaml_belief["pddl_type"].as<int>();
        vector<string> belief_params = retrieveBeliefParams(yaml_belief);
        
        if(belief_pddl_type == Belief().INSTANCE_TYPE)
            return ManagedBelief::buildMBInstance(belief_name, belief_params[0]);
        else if(belief_pddl_type == Belief().PREDICATE_TYPE)
            return ManagedBelief::buildMBPredicate(belief_name, belief_params);
        
        float belief_value;
        if(yaml_belief["value"].IsDefined())
            belief_value = yaml_belief["value"].as<float>();
        
        if(belief_pddl_type == Belief().FUNCTION_TYPE)
            return ManagedBelief::buildMBFunction(belief_name, belief_params, belief_value);
        else 
            return std::nullopt;
    }

    /*
        Given a YAML node which should represent a YAML belief, retrieve its parameters (if any)
    */
    vector<string> retrieveBeliefParams(YAML::Node& yaml_belief)
    {
        vector<string> belief_params;
        if(yaml_belief["params"].IsDefined())
            for(YAML::Node::iterator it_p = yaml_belief["params"].begin(); it_p != yaml_belief["params"].end(); it_p++)
                belief_params.push_back((*it_p).as<string>()); 
        return belief_params;
    }

    /*
        Given a YAML node representing a desire and the name of the condition vector (e.g. "precondition", "context"),
        extract a vector of managed condition
    */
    vector<ManagedCondition> retrieveMGConditions(YAML::Node& yaml_desire, const string& condition_vect_name)
    {
        vector<ManagedCondition> conditions;
        if(yaml_desire[condition_vect_name].IsDefined())//parse precondition
        {
            for(YAML::Node::iterator it_precond = yaml_desire[condition_vect_name].begin(); it_precond != yaml_desire[condition_vect_name].end(); it_precond++)
            {
                auto yaml_precond = (*it_precond);
                string precond_check = yaml_precond["check"].as<string>();//check to be made
                auto yalm_precond_to_check = yaml_precond["condition_to_check"];// a belief -> with it you perform the check
                std::optional<ManagedBelief> opt_mb = parseMGBelief(yalm_precond_to_check);
                if(opt_mb.has_value())
                    conditions.push_back(ManagedCondition{opt_mb.value(), precond_check});
            }
        }
        return conditions;
    }

}// namespace BDIYAMLParser