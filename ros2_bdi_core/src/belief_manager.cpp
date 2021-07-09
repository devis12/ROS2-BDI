#include <cstdlib>
#include <optional>
#include <ctime>
#include <memory>
#include <vector>
#include <set>
#include <mutex>          

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_utils/PDDLBDIConstants.hpp"
#include "ros2_bdi_utils/PDDLBDIConverter.hpp"
#include "ros2_bdi_utils/BDIComparisons.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_COMM_ERRORS 16

using std::string;
using std::vector;
using std::set;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;

using plansys2::ProblemExpertClient;
using plansys2::DomainExpertClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;
using plansys2_msgs::srv::GetProblemInstances;
using std_msgs::msg::Empty;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;

typedef enum {STARTING, SYNC, PAUSE} StateType;                

class BeliefManager : public rclcpp::Node
{
public:
  BeliefManager()
  : rclcpp::Node("belief_manager"), state_(STARTING)
  {
    psys2_comm_errors_ = 0;
    this->declare_parameter("agent_id", "agent0");
    this->declare_parameter("debug", true);
    problem_expert_up_ = false;
  }

  /*
    Init to call at the start, after construction method, to get the node actually started
    initialing problem_expert_ instance, 
    retrieving agent_id_ (thus namespace)
    defining work timer,
    belief set publishers/subscribers callback,
    problem_expert_update notification
  */
  void init()
  { 
    //agent's namespace
    agent_id_ = this->get_parameter("agent_id").as_string();

    //domain expert client to communicate with domain expert node of plansys2
    domain_expert_ = std::make_shared<DomainExpertClient>();

    //problem expert client to communicate with problem expert node of plansys2
    problem_expert_ = std::make_shared<ProblemExpertClient>();

    //Init belief set
    belief_set_ = set<ManagedBelief>();

    //Belief set publisher
    belief_set_publisher_ = this->create_publisher<BeliefSet>("belief_set", 10);
    //Belief to be added notification
    add_belief_subscriber_ = this->create_subscription<Belief>(
                "add_belief", 10,
                bind(&BeliefManager::addBeliefTopicCallBack, this, _1));
    //Belief to be removed notification
    del_belief_subscriber_ = this->create_subscription<Belief>(
                "del_belief", 10,
                bind(&BeliefManager::delBeliefTopicCallBack, this, _1));

    //problem_expert update subscriber
    updated_problem_subscriber_ = this->create_subscription<Empty>(
                "problem_expert/update_notify", 10,
                bind(&BeliefManager::updatedPDDLProblem, this, _1));

    //loop to be called regularly to perform work (publish belief_set_, sync with plansys2 problem_expert node...)
    do_work_timer_ = this->create_wall_timer(
        milliseconds(500),
        bind(&BeliefManager::step, this));

    RCLCPP_INFO(this->get_logger(), "Belief manager node initialized");
  }
  
  /*
    Main loop of work called regularly through a wall timer
  */
  void step()
  {
    // check problem expert alive to init knowledge before spawning
    if(psys2_comm_errors_ > MAX_COMM_ERRORS)
        rclcpp::shutdown();

    switch (state_) {
        
        case STARTING:
        {
            if(isProblemExpertActive()){
                problem_expert_up_ = true;
                psys2_comm_errors_ = 0;
                setState(SYNC);
            }else{
                problem_expert_up_ = false;
                RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with problem expert");
                psys2_comm_errors_++;
            }

            break;
        }

        case SYNC:
        {    
            publishBeliefSet();
        }

        case PAUSE:
        {   
            //RCLCPP_INFO(this->get_logger(), "Not the moment to ask for new cleaning tasks yet");    
            break;
        }
        default:
            break;
    }
  }

private:
    /*  
        Change internal state of the node
    */
    void setState(StateType state)
    {
        state_ = state;
    }


    /*
        Check if the plansys2 problem expert node is up by checking for the presence of one of its services 
        (specifically problem_expert/get_problem_instances) and testing then out by adding and removing instances
        through the cpp api calls
    */
    bool isProblemExpertActive()
    {   
        //check for service to be up
        rclcpp::Client<GetProblemInstances>::SharedPtr client_ = 
            this->create_client<GetProblemInstances>("problem_expert/get_problem_instances");
        if(!client_->wait_for_service(std::chrono::seconds(1))){
            //service seems not even present, just return false
            RCLCPP_WARN(this->get_logger(), "problem_expert/get_problem_instances server does not appear to be up");
            return false;
        }

        //service listed, but is the node actually in an active state? test it with the cpp api (add+remove test instance)
        auto test_name = "bm" + agent_id_ + "_wyp";
        bool isUp = problem_expert_->addInstance(Instance{test_name, "waypoint"});//check if up with test instance
        isUp = isUp && problem_expert_->removeInstance(Instance{test_name, "waypoint"});
        
        return isUp;
    }

    /*
       Publish the current belief set of the agent in agent_id_/belief_set topic
    */
    void publishBeliefSet()
    {
        BeliefSet bset_msg = PDDLBDIConverter::extractBeliefSetMsg(belief_set_);
        belief_set_publisher_->publish(bset_msg);
    }

    /*
        Callback wrt. "problem_expert/update_notify" topic which notifies about any change in the PDDL problem
        update belief set accordingly
    */
    void updatedPDDLProblem(const Empty::SharedPtr msg)
    {
        bool notify;//if anything changes, put it to true
                
        if(this->get_parameter("debug").as_bool())
            RCLCPP_INFO(this->get_logger(), "update problem notification");

        mtx_sync.lock();
            if(problem_expert_up_ || isProblemExpertActive())
            {
                vector<Belief> predicates = PDDLBDIConverter::convertPDDLPredicates(problem_expert_->getPredicates());
                vector<Belief> functions = PDDLBDIConverter::convertPDDLFunctions(problem_expert_->getFunctions());
                notify = updateBeliefSet(predicates, functions);
                
            }
        mtx_sync.unlock();
        if(notify)
            publishBeliefSet();//there has been some modifications, publish new belief set
    }

    
    bool updateBeliefSet(const vector<Belief>& pred_beliefs, const vector<Belief>& fluent_beliefs)
    {
        bool notify;//if anything changes, put it to true
        if(this->get_parameter("debug").as_bool())
            RCLCPP_INFO(this->get_logger(), "update problem: verify if needed to sync (b_set %d, prob_pred %d, prob_fun %d)", 
                belief_set_.size(), pred_beliefs.size(), fluent_beliefs.size());
        //try to add new beliefs or modify current beliefs 
        notify = (pred_beliefs.size() > 0 && addOrModifyBeliefs(pred_beliefs, false));
        notify = (fluent_beliefs.size() > 0 && addOrModifyBeliefs(fluent_beliefs, true)) || notify;

        //check for the removal of some beliefs
        notify = (belief_set_.size() && removedPredicateBeliefs(pred_beliefs)) || notify;
        notify = (belief_set_.size() && removedFluentBeliefs(fluent_beliefs)) || notify;

        return notify;//there has been some modifications
    }

     /*
        Update belief_set checking for presence/absence/alteration of some belief in it wrt. predicates/fluent
        retrieved from the problem_expert
        Returns true if any modification to the belief_set occurs

        @check_for_fluent put to true when you want to check also for modified fluents wrt. their values
    */
    bool addOrModifyBeliefs(const vector<Belief>& beliefs, const bool check_for_fluent)
    {
        bool modified = false;//if anything changes, put it to true

        //check for new or modified (just in case of fluent type) beliefs
        for(auto bel : beliefs)
        {   
            ManagedBelief mb = ManagedBelief{bel};
            int count_bs = belief_set_.count(mb);
            
            if(count_bs == 0)//not found
            {
                RCLCPP_INFO(this->get_logger(), "Adding missing belief ("+mb.type_+"): " + 
                mb.name_ + " " + getParamList(mb) + " (value = " + std::to_string(mb.value_) +")");
                
                addBelief(bel);
                modified = true;//there is an alteration, notify it
            }
            else if(check_for_fluent && bel.value != (*(belief_set_.find(mb))).value_)
            {
                modifyBelief(mb);
                modified = true;//there is an alteration, notify it
            }
        }

        return modified;
    }

    /*
        Extract from the passed beliefs and from the belief set just beliefs of type fluent and check if something
        is in the beliefs array and not in the belief set. If so, remove it from the belief set and notify there has
        been any alteration through the boolean value in return.
    */
    bool removedFluentBeliefs(const vector<Belief>& beliefs)
    {
        bool modified = false;//if anything changes, put it to true

        set<ManagedBelief> fluent_in_belief_set = PDDLBDIConverter::extractMGFluents(belief_set_);
        set<ManagedBelief> fluent_in_pddl_prob = PDDLBDIConverter::extractMGFluents(beliefs);

        if(fluent_in_belief_set.size() > fluent_in_pddl_prob.size())
        {
            //some fluent has to be removed from the belief set, since it has already been removed from the pddl problem
            for(ManagedBelief mb : fluent_in_belief_set)
                if(fluent_in_pddl_prob.count(mb) == 0)
                {
                    modified = true;
                    delBelief(mb);//mb shall be removed from belief_set_ since it's not present in the pddl_problem
                }
        }

        return modified;
    }

    /*
        Extract from the passed beliefs and from the belief set just beliefs of type predicate and check if something
        is in the beliefs array and not in the belief set. If so, remove it from the belief set and notify there has
        been any alteration through the boolean value in return.
    */
    bool removedPredicateBeliefs(const vector<Belief>& beliefs)
    {
        bool modified = false;//if anything changes, put it to true

        set<ManagedBelief> pred_in_belief_set_ = PDDLBDIConverter::extractMGPredicates(belief_set_);
        set<ManagedBelief> pred_in_pddl_prob = PDDLBDIConverter::extractMGPredicates(beliefs);

        if(pred_in_belief_set_.size() > pred_in_pddl_prob.size())
        {
            //some predicate has to be removed from the belief set, since it has already been removed from the pddl problem
            for(ManagedBelief mb : pred_in_belief_set_)
                if(pred_in_belief_set_.count(mb) == 0)
                {
                    modified = true;
                    delBelief(mb);//mb shall be removed from belief_set_ since it's not present in the pddl_problem
                }
        }
        return modified;
    }

    /*
        Get param list as a single joined string separated from spaces
    */
    string getParamList(const ManagedBelief& mb)
    {
        string params_list = "";
        for(int i=0; i<mb.params_.size(); i++)
            params_list += (i==mb.params_.size()-1)? mb.params_[i] : mb.params_[i] + " ";
        return params_list;
    }

    /*
        Build plansys2::Predicate obj from ManagedBelief
    */
    Predicate buildPredicate(const ManagedBelief& mb)
    {
        return Predicate{"(" + mb.name_+ " " + getParamList(mb) + ")"};
    }

    /*
        Build plansys2::Function obj from ManagedBelief
    */
    Function buildFunction(const ManagedBelief& mb)
    {
        return Function{"(" + mb.name_+ " " + getParamList(mb) + std::to_string(mb.value_) + ")"};;
    }

    /*  
        Someone has publish a new belief in the respective topic
    */
    void addBeliefTopicCallBack(const Belief::SharedPtr msg)
    {
        ManagedBelief mb = ManagedBelief{*msg};
        if(this->get_parameter("debug").as_bool())
            RCLCPP_INFO(this->get_logger(), "add_belief callback for " + mb.type_ + ": " + mb.name_ + " " 
                    + getParamList(mb) +  " (value = " + std::to_string(mb.value_) +")");
                        
        mtx_sync.lock();
            if(belief_set_.count(mb)==0)
            {
                if(mb.type_ == PDDLBDIConstants::PREDICATE_TYPE)
                {   
                    //try to add new predicate; if fails, try to check and add missing instances
                    Predicate p_add = buildPredicate(mb);
                    if(problem_expert_->addPredicate(p_add) || 
                        tryAddMissingInstances(mb) && problem_expert_->addPredicate(p_add))
                        addBelief(mb);
                } 
                
                if(mb.type_ == PDDLBDIConstants::FLUENT_TYPE)
                {   
                    //try to add new fluent; if fails, try to check and add missing instances
                    Function f_add =  buildFunction(mb);
                    if(problem_expert_->addFunction(f_add) || 
                        tryAddMissingInstances(mb) && problem_expert_->addFunction(f_add))
                        addBelief(mb);
                }

            }
            else if(mb.type_ == PDDLBDIConstants::FLUENT_TYPE && mb.value_ != (*(belief_set_.find(mb))).value_)
            {
                //fluent present in the belief set with diff. value
                Function f_upd = buildFunction(mb);
                if(problem_expert_->updateFunction(f_upd))//instances have to be already present
                    modifyBelief(mb);
            }
        mtx_sync.unlock();

    }

    /*
        Create array of boolean flags denoting missing instances' positions
        wrt. parameters in the passed ManagedBelief argument
    */
    vector<bool> computeMissingInstancesPos(const ManagedBelief& mb)
    {
        vector<bool> missing_pos = vector<bool>();
        vector<Instance> instances = problem_expert_->getInstances();
        for(string mb_par : mb.params_)
        {    
            bool found = false;

            for(Instance ins : instances)
            {
                if(mb_par == ins.name)
                {
                    found = true;//instance already defined
                    break;
                }
            }

            missing_pos.push_back(!found);//flag denote missing instance
        }
        return missing_pos;
    }

    /*
        Try adding missing instances (if any)
    */
    bool tryAddMissingInstances(const ManagedBelief& mb)
    {   
        vector<bool> missing_pos = computeMissingInstancesPos(mb);
        
        if(this->get_parameter("debug").as_bool())
        {
            string missing_pos_string = "";
            for(bool mp : missing_pos)
                missing_pos_string += std::to_string((int)mp) + ", ";
            RCLCPP_INFO(this->get_logger(), "Missing: " + missing_pos_string);
        }
        
        
        if(mb.type_ == PDDLBDIConstants::PREDICATE_TYPE)
        {   
            try {
                //retrieve from domain expert definition information about this predicate
                Predicate pred = domain_expert_->getPredicate(mb.name_).value();
                for(int i = 0; i<pred.parameters.size(); i++)
                {
                    if(missing_pos[i])//missing instance
                    {   
                        if(this->get_parameter("debug").as_bool())
                            RCLCPP_INFO(this->get_logger(), "Trying to add instance: " + mb.params_[i] + " - " + pred.parameters[i].type);
                        if(!problem_expert_->addInstance(Instance{mb.params_[i], pred.parameters[i].type}))//add instance (type found from domain expert)
                            return false;//add instance failed
                    }
                }

            } catch(const std::bad_optional_access& e) {}
        } 
        
        if(mb.type_ == PDDLBDIConstants::FLUENT_TYPE)
        {   
            try {
                //retrieve from domain expert definition information about this function
                Function function = domain_expert_->getFunction(mb.name_).value();
                for(int i = 0; i<function.parameters.size(); i++)
                {
                    if(missing_pos[i])//missing instance
                    {
                        if(this->get_parameter("debug").as_bool())
                            RCLCPP_INFO(this->get_logger(), "Trying to add instance: " + mb.params_[i] + " - " + function.parameters[i].type);
                        if(!problem_expert_->addInstance(Instance{mb.params_[i], function.parameters[i].type}))//add instance (type found from domain expert)
                            return false;//add instance failed
                    }

                }

            } catch(const std::bad_optional_access& e) {}
        }

        return true;
    }

    /*  
        Someone has publish a belief to be removed in the respective topic
    */
    void delBeliefTopicCallBack(const Belief::SharedPtr msg)
    {
        ManagedBelief mb = ManagedBelief{*msg};
        if(this->get_parameter("debug").as_bool())
            RCLCPP_INFO(this->get_logger(), "del_belief callback for " + mb.type_ + ": " + mb.name_ + " " 
                    + getParamList(mb) +  " (value = " + std::to_string(mb.value_) +")");
        bool done = false;
        mtx_sync.lock();
            if(belief_set_.count(mb)==1)
            {
                if(mb.type_ == PDDLBDIConstants::PREDICATE_TYPE)
                {
                    Predicate pred = buildPredicate(mb);
                    done = !problem_expert_->existPredicate(pred) || problem_expert_->removePredicate(pred);
                }

                if(mb.type_ == PDDLBDIConstants::FLUENT_TYPE)
                {
                    Function fun = buildFunction(mb);
                    done = !problem_expert_->existFunction(fun) || problem_expert_->removeFunction(fun);
                }
                
                if(done)
                    belief_set_.erase(mb);
            }
        mtx_sync.unlock();
    }

    /*
        add belief into belief set
    */
    void addBelief(const ManagedBelief& mgbelief)
    {
        belief_set_.insert(mgbelief);
        if(this->get_parameter("debug").as_bool())
            RCLCPP_INFO(this->get_logger(), "Added belief ("+mgbelief.type_+"): " + 
                mgbelief.name_ + " " + getParamList(mgbelief) + " (value = " + std::to_string(mgbelief.value_) +")");
                       
    }

    /*
        remove and add belief into belief set 
        (i.e. cover the case of same fluent with diff. values)
    */
    void modifyBelief(const ManagedBelief& mgbelief)
    {
        if(belief_set_.count(mgbelief) == 1){
            belief_set_.erase(mgbelief);
            belief_set_.insert(mgbelief);
            if(this->get_parameter("debug").as_bool())
                RCLCPP_INFO(this->get_logger(), "Modified belief ("+mgbelief.type_+"): " + 
                    mgbelief.name_ + " " + getParamList(mgbelief) + " (value = " + std::to_string(mgbelief.value_) +")");
        }
    }

    /*
        delete belief from belief set
    */
    void delBelief(const ManagedBelief& mgbelief)
    {
        belief_set_.erase(mgbelief);
        if(this->get_parameter("debug").as_bool())
            RCLCPP_INFO(this->get_logger(), "Removed belief ("+mgbelief.type_+"): " + 
                mgbelief.name_ + " " + getParamList(mgbelief) + " (value = " + std::to_string(mgbelief.value_) +")");
    }
    

    // internal state of the node
    StateType state_;
       
    //mutex for deciding in which direction we're sync (PDDL->belief_set_ or belief_set_->PDDL)
    std::mutex mtx_sync;        
    
    // agent id that defines the namespace in which the node operates
    string agent_id_;
    // callback to perform main loop of work regularly
    rclcpp::TimerBase::SharedPtr do_work_timer_;

    // counter of communication errors with plansys2
    int psys2_comm_errors_;
    // problem expert instance to call the problem expert api
    std::shared_ptr<ProblemExpertClient> problem_expert_;
    // domain expert instance to call the problem expert api
    std::shared_ptr<DomainExpertClient> domain_expert_;
    // flag to denote if the problem expert node seems to correctly answer 
    bool problem_expert_up_;

    // belief set of the agent <agent_id_>
    set<ManagedBelief> belief_set_;

    // belief set publishers/subscribers
    rclcpp::Subscription<Belief>::SharedPtr add_belief_subscriber_;//add belief notify on topic
    rclcpp::Subscription<Belief>::SharedPtr del_belief_subscriber_;//del belief notify on topic
    rclcpp::Publisher<BeliefSet>::SharedPtr belief_set_publisher_;//belief set publisher

    // plansys2 problem expert notification for updates
    rclcpp::Subscription<Empty>::SharedPtr updated_problem_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BeliefManager>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
