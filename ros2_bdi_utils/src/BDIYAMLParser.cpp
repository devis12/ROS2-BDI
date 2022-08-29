#include "ros2_bdi_utils/BDIYAMLParser.hpp"

using std::string;
using std::vector;
using std::set;
using std::optional;

using plansys2_msgs::msg::Param;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;

using BDIManaged::ManagedParam;
using BDIManaged::ManagedBelief;
using BDIManaged::ManagedDesire;
using BDIManaged::ManagedCondition;
using BDIManaged::ManagedConditionsConjunction;
using BDIManaged::ManagedConditionsDNF;
using BDIManaged::ManagedReactiveRule;

namespace BDIYAMLParser
{
    /*
        Extract managed beliefs from a YAML file containing them
        throws YAML::InvalidNode, YAML::BadFile, YAML::BadConversion
    */
    vector<ManagedBelief> extractMGBeliefs(const string& bset_filepath, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert)
    {
        YAML::Node mybset = YAML::LoadFile(bset_filepath);
        return parseMGBeliefs(mybset, domain_expert);
    }

    /*
        Given a YAML Node which should represent an array of beliefs, parse it and build a vector<ManagedBelief>
        return empty if there isn't any belief available within the node
    */
    vector<ManagedBelief> parseMGBeliefs(YAML::Node& yaml_beliefs, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert)
    {
        return parseMGBeliefs(yaml_beliefs, Belief().ALL_TYPE, domain_expert);
    }

    /*
        Extract managed reactive rules from a YAML file containing them
    */
    set<ManagedReactiveRule> extractMGReactiveRules(const string& mgrrules_filepath, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert)
    {
        YAML::Node my_rrules_yaml = YAML::LoadFile(mgrrules_filepath);
        return parseMGReactiveRules(my_rrules_yaml, domain_expert);
    }   
    
    /*
        Parse and get managed reactive rules from a YAML node containing them
    */
    set<ManagedReactiveRule> parseMGReactiveRules(YAML::Node& yaml_rrules, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert)
    {
        set<ManagedReactiveRule> rrules;
        uint8_t ai_id_counter = 1;
        for(YAML::Node::iterator it = yaml_rrules.begin(); it != yaml_rrules.end(); it++)
        {
            auto yaml_rrule = (*it);

            if(yaml_rrule["condition"].IsDefined() && yaml_rrule["reactive_rules"].IsDefined())
            {
                ManagedConditionsDNF dnf_condition = parseMGConditionsDNF(yaml_rrule, "condition",domain_expert);
                set<MGBeliefOp> bset_effects = set<std::pair<ReactiveOp, BDIManaged::ManagedBelief>>();
                set<MGDesireOp> dset_effects = set<std::pair<ReactiveOp, BDIManaged::ManagedDesire>>();
                auto yaml_rrules_effects = yaml_rrule["reactive_rules"];
                for(YAML::Node::iterator it_eff = yaml_rrules_effects.begin(); it_eff != yaml_rrules_effects.end(); it_eff++)
                {
                    auto rrule_effect = (*it_eff);
                    if(rrule_effect["set"].as<string>() == "belief")
                    {
                        //bset operation
                        auto yaml_belief = rrule_effect["value"];
                        std::optional<ManagedBelief> mg_belief = parseMGBelief(yaml_belief, domain_expert);
                        string op_s = rrule_effect["operation"].as<string>();
                        if((op_s == "ADD" || op_s == "DEL") && mg_belief.has_value())
                        {   
                            ReactiveOp op = (op_s == "ADD")? ReactiveOp::ADD : ReactiveOp::DEL;
                            bset_effects.insert(std::make_pair(op, mg_belief.value()));
                        }
                    }
                    else if(rrule_effect["set"].as<string>() == "desire")
                    {
                        //dset operation
                        auto yaml_desire = rrule_effect["value"];
                        std::optional<ManagedDesire> mg_desire = parseMGDesire(yaml_desire, domain_expert);
                        string op_s = rrule_effect["operation"].as<string>();
                        if((op_s == "ADD" || op_s == "DEL") && mg_desire.has_value())
                        {   
                            ReactiveOp op = (op_s == "ADD")? ReactiveOp::ADD : ReactiveOp::DEL;
                            dset_effects.insert(std::make_pair(op, mg_desire.value()));
                        }
                    }
                }

                if(bset_effects.size() > 0 || dset_effects.size() > 0)//if any effect were registered for the given condition insert it to the set of MGReactiveRules
                {
                    rrules.insert(ManagedReactiveRule{ai_id_counter, dnf_condition, bset_effects, dset_effects});
                    ai_id_counter++;
                }
            }
            
        }

        return rrules;
    }

    /*
        Given a YAML Node which should represent an array of beliefs, parse it and build a vector<ManagedBelief>
        containing just the beliefs of the given type (if ALL_TYPE, do not filter, returns all beliefs of any given/valid type)
        return empty if there isn't any belief available within the node
    */
    vector<ManagedBelief> parseMGBeliefs(YAML::Node& yaml_beliefs, const int& belief_type, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert)
    {
        vector<ManagedBelief> mgBeliefs;

        for(YAML::Node::iterator it = yaml_beliefs.begin(); it != yaml_beliefs.end(); it++)
        {
            auto yaml_belief = (*it);
            std::optional<ManagedBelief> opt_mb = parseMGBelief(yaml_belief, domain_expert);

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
    std::optional<ManagedBelief> parseMGBelief(YAML::Node& yaml_belief, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert)
    {
        string belief_name = yaml_belief["name"].as<string>();
        int belief_pddl_type = yaml_belief["pddl_type"].as<int>();
        
        string type;
        vector<ManagedParam> belief_params;
        if(belief_pddl_type == Belief().PREDICATE_TYPE)
        {
            std::optional<plansys2::Predicate> pred_def = domain_expert->getPredicate(belief_name); 
            if(!pred_def.has_value())
                return std::nullopt;
            else
                belief_params = parseBeliefParams(yaml_belief, pred_def.value().parameters);
        }
        else if(belief_pddl_type == Belief().FUNCTION_TYPE)
        {
            std::optional<plansys2::Function> func_def = domain_expert->getFunction(belief_name);
            if(!func_def.has_value())
                return std::nullopt;
            else
                belief_params = parseBeliefParams(yaml_belief, func_def.value().parameters);

        }
        else if(belief_pddl_type == Belief().INSTANCE_TYPE)
            type = yaml_belief["type"].IsDefined()? 
                        yaml_belief["type"].as<string>() 
                    : 
                        belief_params.size() == 1? belief_params[0].name : "";

        if(belief_pddl_type == Belief().INSTANCE_TYPE)
            return ManagedBelief::buildMBInstance(belief_name, type);
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
    vector<ManagedParam> parseBeliefParams(YAML::Node& yaml_belief, const std::vector<plansys2_msgs::msg::Param>& params_def)
    {
        vector<ManagedParam> belief_params;
            if(yaml_belief["params"].IsDefined())
            {
                int16_t i = 0;
                for(YAML::Node::iterator it_p = yaml_belief["params"].begin(); it_p != yaml_belief["params"].end(); it_p++)
                {
                    string param_name = (*it_p).as<string>(); 
                    if(i<params_def.size())
                    {
                        // match param name with type
                        belief_params.push_back(ManagedParam{param_name, params_def[i].type});
                    }
                    i++;
                }
            }
        return belief_params;
    }


    /*
        Extract managed desires from a YAML file containing them
    */
    vector<ManagedDesire> extractMGDesires(const string& dset_filepath, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert)
    {
        vector<ManagedDesire> mgDesires;
        YAML::Node mydset = YAML::LoadFile(dset_filepath);
        for(YAML::Node::iterator it = mydset.begin(); it != mydset.end(); it++)
        {
            auto yaml_desire = (*it);
            std::optional<ManagedDesire> opt_md = parseMGDesire(yaml_desire, domain_expert);
                    
            if(opt_md.has_value())
                mgDesires.push_back(opt_md.value());
        }       
        return mgDesires;
    }
    

    /*
        Given a YAML Node which should represent a desire, parse it and build a ManagedDesire
        return std::nullopt if not possible
    */
    std::optional<ManagedDesire> parseMGDesire(YAML::Node& yaml_desire, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert)
    {
        string desire_name = yaml_desire["name"].as<string>();
        float desire_deadline = yaml_desire["deadline"].as<float>();
        if(desire_deadline < 0)
            desire_deadline = 0.0f;
        float desire_priority = yaml_desire["priority"].as<float>();
        if(desire_priority < 0 || desire_priority > 1)
            desire_priority = 0.0f;
        
        auto yaml_desire_value = yaml_desire["value"];
        vector<ManagedBelief> desire_value = parseMGBeliefs(yaml_desire_value, Belief().PREDICATE_TYPE, domain_expert);
        
        // retrieving preconditions and context (if there is any, otherwise empty)
        ManagedConditionsDNF preconditions = parseMGConditionsDNF(yaml_desire, "precondition", domain_expert);
        ManagedConditionsDNF context = parseMGConditionsDNF(yaml_desire, "context", domain_expert);

        vector<ManagedBelief> rollback_belief_add;       
        if(yaml_desire["rollback_belief_add"].IsDefined())
        {   
            auto yaml_rb_beliefs_add = yaml_desire["rollback_belief_add"];
            rollback_belief_add = parseMGBeliefs(yaml_rb_beliefs_add, domain_expert);
        }

        vector<ManagedBelief> rollback_belief_del;       
        if(yaml_desire["rollback_belief_add"].IsDefined())
        {
            auto yaml_rb_beliefs_del = yaml_desire["rollback_belief_del"];
            rollback_belief_del = parseMGBeliefs(yaml_rb_beliefs_del, domain_expert);
        }

        if(desire_value.size() > 0)
            return(ManagedDesire{desire_name, desire_value, 
                desire_priority, desire_deadline, preconditions, context, rollback_belief_add, rollback_belief_del});
        else
            return std::nullopt;
    }

    /*
        Given a YAML parent node containing a condition expressed in DNF and the name of the condition vector (e.g. "precondition", "context"),
        extract a vector of managed condition DNF clause
    */
    ManagedConditionsDNF parseMGConditionsDNF(YAML::Node& yaml_parent_node, const string& condition_vect_name, const std::shared_ptr<plansys2::DomainExpertClient>& domain_expert)
    {
        
        vector<ManagedConditionsConjunction> mg_clauses;

        if(yaml_parent_node[condition_vect_name].IsDefined()
            && yaml_parent_node[condition_vect_name]["clauses"].IsDefined()
            && yaml_parent_node[condition_vect_name]["clauses"][0]["literals"].IsDefined())//parse precondition
        {
            auto yaml_clauses = yaml_parent_node[condition_vect_name]["clauses"];

            for(YAML::Node::iterator it_clause = yaml_clauses.begin(); it_clause != yaml_clauses.end(); it_clause++)
            {
                auto yaml_clause = (*it_clause);
                vector<ManagedCondition> literals;
                for(YAML::Node::iterator it_literal = yaml_clause["literals"].begin(); it_literal != yaml_clause["literals"].end(); it_literal++)
                {
                    auto yaml_cond = (*it_literal);
                    string precond_check = yaml_cond["check"].as<string>();//check to be made
                    auto yalm_cond_to_check = yaml_cond["condition_to_check"];// a belief -> with it you perform the check
                    std::optional<ManagedBelief> opt_mb = parseMGBelief(yalm_cond_to_check, domain_expert);
                    if(opt_mb.has_value())
                        literals.push_back(ManagedCondition{opt_mb.value(), precond_check});
                }
                mg_clauses.push_back(ManagedConditionsConjunction{literals});
            }
        }
        return ManagedConditionsDNF{mg_clauses};
    }

}// namespace BDIYAMLParser