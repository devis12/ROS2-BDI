#ifndef PLANSYS_MONITOR_PARAMS_H_
#define PLANSYS_MONITOR_PARAMS_H_

/* Parameters affecting internal logic (recompiling required) */
#define TIMER_MIN 250
#define TIMER_MAX 20000

#define PSYS2NODES 4
#define PSYS2_CK_STATE_SRV "get_state"

#define PSYS_STATE_TOPIC "plansys_state"

// psys2 nodes' names
#define PSYS2_DOM_EXPERT "domain_expert"
#define PSYS2_PROB_EXPERT "problem_expert"
#define PSYS2_PLANNER "planner"
#define JAVAFF_PLANNER "javaff_server"
#define PSYS2_EXECUTOR "executor"

// index for the node and client callers
#define PSYS2_DOM_EXPERT_I 0
#define PSYS2_PROB_EXPERT_I 1
#define PSYS2_PLANNER_I 2
#define PSYS2_EXECUTOR_I 3

//seconds to wait before giving up on performing a PSYS2 get_state request (service does not appear to be up)
#define WAIT_GET_STATE_SRV_UP 1   

//seconds to wait before giving up on waiting for the PSYS2 get_state response
#define WAIT_GET_STATE_RESPONSE_TIMEOUT 1

#endif