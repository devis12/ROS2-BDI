#ifndef PLAN_DIRECTOR_PARAMS_H_
#define PLAN_DIRECTOR_PARAMS_H_

/* Parameters affecting internal logic for Plan Director node (recompiling required) */
#define NO_PLAN_INTERVAL 1000
#define PLAN_INTERVAL 250

#define PLAN_DIRECTOR_NODE_NAME "plan_director"
#define PLAN_EXECUTION_SRV "plan_execution"
#define PLAN_EXECUTION_TOPIC "plan_execution_info"

/* ROS2 Parameter names for Plan Director node */
#define PARAM_CANCEL_AFTER_DEADLINE "rtc_deadline"

#define DEFAULT_VAL_CANCEL_AFTER_DEADLINE 2.0

#endif