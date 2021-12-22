#ifndef SCHEDULER_PARAMS_H_
#define SCHEDULER_PARAMS_H_

/* Parameters affecting internal logic (recompiling required) */
#define SCHEDULER_NODE_NAME "scheduler"
#define VAL_RESCHEDULE_POLICY_NO_IF_EXEC "NO_PREEMPT"
#define VAL_RESCHEDULE_POLICY_IF_EXEC "PREEMPT"

#define DESIRE_SET_TOPIC "desire_set"
#define ADD_DESIRE_TOPIC "add_desire"
#define DEL_DESIRE_TOPIC "del_desire"

#define INIT_DESIRE_SET_FILENAME "init_dset.yaml"

/* ROS2 Parameter names for PlanSys2Monitor node */
#define PARAM_MAX_TRIES_COMP_PLAN "comp_plan_tries"
#define PARAM_MAX_TRIES_EXEC_PLAN "exec_plan_tries"
#define PARAM_RESCHEDULE_POLICY "reschedule_policy"
#define PARAM_AUTOSUBMIT_PREC "autosub_prec"
#define PARAM_AUTOSUBMIT_CONTEXT "autosub_context"

#endif