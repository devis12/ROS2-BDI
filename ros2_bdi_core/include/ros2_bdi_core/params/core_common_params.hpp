#ifndef CORE_COMMON_PARAMS_H_
#define CORE_COMMON_PARAMS_H_

/* ROS2 Parameter names for ROS2-BDI core nodes */
#define PARAM_AGENT_ID "agent_id"
#define PARAM_DEBUG "debug"
#define PARAM_AGENT_GROUP_ID "agent_group"
#define MAX_COMM_ERRORS 16

#define BELIEF_MANAGER_NODE_NAME "belief_manager"
#define EVENT_LISTENER_NODE_NAME "event_listener"
#define MA_REQUEST_HANDLER_NODE_NAME "ma_request_handler"
#define PLAN_DIRECTOR_NODE_NAME "plan_director"
#define PSYS_MONITOR_NODE_NAME "plansys_monitor"
#define SCHEDULER_NODE_NAME "scheduler"

#define PARAM_PLANNING_MODE "planning_mode"

#define PLANNING_MODE_OFFLINE "offline"
#define PLANNING_MODE_ONLINE "online"

#define LIFECYCLE_STATUS_TOPIC "lifecycle_status"

#endif