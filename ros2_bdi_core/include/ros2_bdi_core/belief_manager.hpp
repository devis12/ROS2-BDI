#ifndef BELIEF_MANAGER_H_
#define BELIEF_MANAGER_H_

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <mutex>  

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/plan_sys2_state.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"

#include "ros2_bdi_core/params/belief_manager_params.hpp"
#include "ros2_bdi_core/support/plansys2_monitor_client.hpp"

#include "std_msgs/msg/empty.hpp"
#include "rclcpp/rclcpp.hpp"

typedef enum {STARTING, SYNC, PAUSE} StateType;                

class BeliefManager : public rclcpp::Node
{
    public:

        /* Constructor method */
        BeliefManager();

        /*
            Init to call at the start, after construction method, to get the node actually started
            initialing problem_expert_ instance, 
            retrieving agent_id_ (thus namespace)
            defining work timer,
            belief set publishers/subscribers callback,
            problem_expert_update notification
        */
        void init();
        
        /*
            Main loop of work called regularly through a wall timer
        */
        void step();

        /*
            Wait for PlanSys2 to boot at best for max_wait
        */
        bool wait_psys2_boot(const std::chrono::seconds max_wait = std::chrono::seconds(16))
        {
            psys2_monitor_client_ = std::make_shared<PlanSys2MonitorClient>(BELIEF_MANAGER_NODE_NAME + std::string("_psys2caller_"));
            return psys2_monitor_client_->areAllPsysNodeActive(max_wait);
        }

    private:
        /*  
            Change internal state of the node
        */
        void setState(StateType state){ state_ = state; }

        /*
            Received notification about PlanSys2 nodes state by plansys2 monitor node
        */
        void callbackPsys2State(const ros2_bdi_interfaces::msg::PlanSys2State::SharedPtr msg);

        /*
            Publish the current belief set of the agent in agent_id_/belief_set topic
        */
        void publishBeliefSet();

        /*
            Expect to find yaml file to init the belief set in "/tmp/{agent_id}/init_bset.yaml"
        */
        void tryInitBeliefSet();

        /*
            Callback wrt. "problem_expert/update_notify" topic which notifies about any change in the PDDL problem
            update belief set accordingly
        */
        void updatedPDDLProblem(const std_msgs::msg::Empty::SharedPtr msg);

        
        bool updateBeliefSet(const std::vector<ros2_bdi_interfaces::msg::Belief>& ins_beliefs, 
            const std::vector<ros2_bdi_interfaces::msg::Belief>& pred_beliefs, const std::vector<ros2_bdi_interfaces::msg::Belief>& fun_beliefs);

        /*
            Update belief_set checking for presence/absence/alteration of some belief in it wrt. predicates/function
            retrieved from the problem_expert
            Returns true if any modification to the belief_set occurs

            @check_for_function put to true when you want to check also for modified functions wrt. their values
        */
        bool addOrModifyBeliefs(const std::vector<ros2_bdi_interfaces::msg::Belief>& beliefs, const bool check_for_function);

        /*
            Extract from the passed beliefs and from the belief set just beliefs of type instance and check if something
            is in the beliefs array and not in the belief set. If so, remove it from the belief set and notify there has
            been any alteration through the boolean value in return.
        */
        bool removedInstanceBeliefs(const std::vector<ros2_bdi_interfaces::msg::Belief>& beliefs);
        
        /*
            Extract from the passed beliefs and from the belief set just beliefs of type function and check if something
            is in the beliefs array and not in the belief set. If so, remove it from the belief set and notify there has
            been any alteration through the boolean value in return.
        */
        bool removedFunctionBeliefs(const std::vector<ros2_bdi_interfaces::msg::Belief>& beliefs);

        /*
            Extract from the passed beliefs and from the belief set just beliefs of type predicate and check if something
            is in the beliefs array and not in the belief set. If so, remove it from the belief set and notify there has
            been any alteration through the boolean value in return.
        */
        bool removedPredicateBeliefs(const std::vector<ros2_bdi_interfaces::msg::Belief>& beliefs);

        /*  
            Someone has publish a new belief in the respective topic
        */
        void addBeliefTopicCallBack(const ros2_bdi_interfaces::msg::Belief::SharedPtr msg);

        /*
            Add Belief in the belief set, just after having appropriately sync the pddl_problem to add it there too
        */
        void addBeliefSyncPDDL(const BDIManaged::ManagedBelief& mb);

        /*
            Create array of boolean flags denoting missing instances' positions
            wrt. parameters in the passed ManagedBelief argument
        */
        std::vector<bool> computeMissingInstancesPos(const BDIManaged::ManagedBelief& mb);

        /*
            Try adding missing instances (if any)
        */
        bool tryAddMissingInstances(const BDIManaged::ManagedBelief& mb);

        /*  
            Someone has publish a belief to be removed in the respective topic
        */
        void delBeliefTopicCallBack(const ros2_bdi_interfaces::msg::Belief::SharedPtr msg);


        /*
            Remove Belief from the belief set, just after having appropriately sync the pddl_problem to remove it from there too
        */
        void delBeliefSyncPDDL(const BDIManaged::ManagedBelief& mb);

        /*
            add belief into belief set
        */
        void addBelief(const BDIManaged::ManagedBelief& mb);

        /*
            remove and add belief into belief set 
            (i.e. cover the case of same function with diff. values)
        */
        void modifyBelief(const BDIManaged::ManagedBelief& mb);

        /*
            delete belief from belief set
        */
        void delBelief(const BDIManaged::ManagedBelief& mb);
        

        // internal state of the node
        StateType state_;
        
        //mutex for deciding in which direction we're sync (PDDL->belief_set_ or belief_set_->PDDL)
        std::mutex mtx_sync;        
        
        // agent id that defines the namespace in which the node operates
        std::string agent_id_;
        // callback to perform main loop of work regularly
        rclcpp::TimerBase::SharedPtr do_work_timer_;

        // counter of communication errors with plansys2
        int psys2_comm_errors_;
        // problem expert instance to call the problem expert api
        std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
        // domain expert instance to call the problem expert api
        std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
        // contain last pddl problem string known at the moment (goal part stripped away)
        std::string last_pddl_problem_;
        
        
        // flag to denote if the problem expert node seems to be up and active
        bool psys2_problem_expert_active_;
        // flag to denote if the domain expert node seems to be up and active
        bool psys2_domain_expert_active_;
        // plansys2 node status monitor subscription
        rclcpp::Subscription<ros2_bdi_interfaces::msg::PlanSys2State>::SharedPtr plansys2_status_subscriber_;
        
        // belief set has been init. (or at least the process to do so has been tried)
        bool init_bset_;

        // belief set of the agent <agent_id_>
        std::set<BDIManaged::ManagedBelief> belief_set_;

        // belief set publishers/subscribers
        rclcpp::Subscription<ros2_bdi_interfaces::msg::Belief>::SharedPtr add_belief_subscriber_;//add belief notify on topic
        rclcpp::Subscription<ros2_bdi_interfaces::msg::Belief>::SharedPtr del_belief_subscriber_;//del belief notify on topic
        rclcpp::Publisher<ros2_bdi_interfaces::msg::BeliefSet>::SharedPtr belief_set_publisher_;//belief set publisher

        // plansys2 problem expert notification for updates
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr updated_problem_subscriber_;

        // PlanSys2 Monitor Client supporting nodes & clients for calling the {psys2_node}/get_state services
        std::shared_ptr<PlanSys2MonitorClient> psys2_monitor_client_;

}; //BeliefManager class prototype

#endif //BELIEF_MANAGER_H_
