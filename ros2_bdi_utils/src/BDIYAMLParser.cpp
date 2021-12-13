#include "ros2_bdi_utils/BDIYAMLParser.hpp"

using std::string;
using std::vector;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;

using BDIManaged::ManagedBelief;
using BDIManaged::ManagedDesire;
using BDIManaged::ManagedCondition;
using BDIManaged::ManagedConditionsConjunction;
using BDIManaged::ManagedConditionsDNF;

namespace BDIYAMLParser
{
    /*
        Extract managed beliefs from a YAML file containing them
        throws YAML::InvalidNode, YAML::BadFile, YAML::BadConversion
    */
    vector<ManagedBelief> extractMGBeliefs(const string& bset_filepath)
    {
        YAML::Node mybset = YAML::LoadFile(bset_filepath);
        return parseMGBeliefs(mybset);
    }

    /*
        Given a YAML Node which should represent an array of beliefs, parse it and build a vector<ManagedBelief>
        return empty if there isn't any belief available within the node
    */
    vector<ManagedBelief> parseMGBeliefs(YAML::Node& yaml_beliefs)
    {
        return parseMGBeliefs(yaml_beliefs, Belief().ALL_TYPE);
    }

    /*
        Given a YAML Node which should represent an array of beliefs, parse it and build a vector<ManagedBelief>
        containing just the beliefs of the given type (if ALL_TYPE, do not filter, returns all beliefs of any given/valid type)
        return empty if there isn't any belief available within the node
    */
    vector<ManagedBelief> parseMGBeliefs(YAML::Node& yaml_beliefs, const int& belief_type)
    {
        vector<ManagedBelief> mgBeliefs;

        for(YAML::Node::iterator it = yaml_beliefs.begin(); it != yaml_beliefs.end(); it++)
        {
            auto yaml_belief = (*it);
            std::optional<ManagedBelief> opt_mb = parseMGBelief(yaml_belief);

            if(opt_mb.has_value())
            {
                ManagedBelief mb = opt_mb.value();
                // filter for passed type
                if(belief_type == Belief().ALL_TYPE || mb.pddlType() == belief_type)//if ALL_TYPE (do not filter)
                    mgBeliefs.push_back(mb);
            }
        }

        return mgBeliefs;
    }

    /*
        Given a YAML Node which should represent a belief, parse it and build a ManagedBelief
        return std::nullopt if not possible
    */
    std::optional<ManagedBelief> parseMGBelief(YAML::Node& yaml_belief)
    {
        string belief_name = yaml_belief["name"].as<string>();
        int belief_pddl_type = yaml_belief["pddl_type"].as<int>();
        vector<string> belief_params = parseBeliefParams(yaml_belief);
        
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
    vector<string> parseBeliefParams(YAML::Node& yaml_belief)
    {
        vector<string> belief_params;
        if(yaml_belief["params"].IsDefined())
            for(YAML::Node::iterator it_p = yaml_belief["params"].begin(); it_p != yaml_belief["params"].end(); it_p++)
                belief_params.push_back((*it_p).as<string>()); 
        return belief_params;
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
        
        auto yaml_desire_value = yaml_desire["value"];
        vector<ManagedBelief> desire_value = parseMGBeliefs(yaml_desire_value, Belief().PREDICATE_TYPE);
        
        // retrieving preconditions and context (if there is any, otherwise empty)
        ManagedConditionsDNF preconditions = parseMGConditionsDNF(yaml_desire, "precondition");
        ManagedConditionsDNF context = parseMGConditionsDNF(yaml_desire, "context");

        vector<ManagedBelief> rollback_belief_add;       
        if(yaml_desire["rollback_belief_add"].IsDefined())
        {   
            auto yaml_rb_beliefs_add = yaml_desire["rollback_belief_add"];
            rollback_belief_add = parseMGBeliefs(yaml_rb_beliefs_add);
        }

        vector<ManagedBelief> rollback_belief_del;       
        if(yaml_desire["rollback_belief_add"].IsDefined())
        {
            auto yaml_rb_beliefs_del = yaml_desire["rollback_belief_del"];
            rollback_belief_del = parseMGBeliefs(yaml_rb_beliefs_del);
        }

        if(desire_value.size() > 0)
            return(ManagedDesire{desire_name, desire_value, 
                desire_priority, desire_deadline, preconditions, context, rollback_belief_add, rollback_belief_del});
        else
            return std::nullopt;
    }

    /*
        Given a YAML node representing a desire and the name of the condition vector (e.g. "precondition", "context"),
        extract a vector of managed condition DNF clause
    */
    ManagedConditionsDNF parseMGConditionsDNF(YAML::Node& yaml_desire, const string& condition_vect_name)
    {
        
        vector<ManagedConditionsConjunction> mg_clauses;

        if(yaml_desire[condition_vect_name].IsDefined()
            && yaml_desire[condition_vect_name]["clauses"].IsDefined()
            && yaml_desire[condition_vect_name]["clauses"][0]["literals"].IsDefined())//parse precondition
        {
            auto yaml_clauses = yaml_desire[condition_vect_name]["clauses"];

            for(YAML::Node::iterator it_clause = yaml_clauses.begin(); it_clause != yaml_clauses.end(); it_clause++)
            {
                auto yaml_clause = (*it_clause);
                vector<ManagedCondition> literals;
                for(YAML::Node::iterator it_literal = yaml_clause["literals"].begin(); it_literal != yaml_clause["literals"].end(); it_literal++)
                {
                    auto yaml_cond = (*it_literal);
                    string precond_check = yaml_cond["check"].as<string>();//check to be made
                    auto yalm_cond_to_check = yaml_cond["condition_to_check"];// a belief -> with it you perform the check
                    std::optional<ManagedBelief> opt_mb = parseMGBelief(yalm_cond_to_check);
                    if(opt_mb.has_value())
                        literals.push_back(ManagedCondition{opt_mb.value(), precond_check});
                }
                mg_clauses.push_back(ManagedConditionsConjunction{literals});
            }
        }
        return mg_clauses;
    }

}// namespace BDIYAMLParser