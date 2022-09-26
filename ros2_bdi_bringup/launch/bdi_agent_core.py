from launch_ros.actions import Node

# Bringup parameters
from bringup_params import *


'''
    Log automatically set parameters with their default values
'''
def log_automatic_set(param, param_value):
    print('Invalid or unspecified value for \"{}\" which will be automatically set to {}'.format(param, param_value))

'''
    Utility function for a strong type check on a simple py List
'''
def is_list_of(l, type):
    if not isinstance(l, list):
            return False
    else:
        for el in l:
            if not isinstance(el, type):
                return False
        return True




'''
    PlanSys2Monitor Node builder
'''
def build_PlanSysMonitor(namespace, agent_id, init_params):
    debug = (DEBUG_ACTIVE_NODES_PARAM in init_params) and ('plansys_monitor' in init_params[DEBUG_ACTIVE_NODES_PARAM])
    planning_mode = 'offline'
    if PLANNING_MODE_PARAM in init_params:
        planning_mode = init_params[PLANNING_MODE_PARAM] if init_params[PLANNING_MODE_PARAM] in ['offline', 'online'] else 'offline'
    
    return Node(
        package='ros2_bdi_core',
        executable='plansys_monitor',
        name='plansys_monitor',
        namespace=namespace,
        output='screen',
        parameters=[ {AGENT_ID_PARAM: agent_id}, {DEBUG_PARAM: debug}, {PLANNING_MODE_PARAM: planning_mode}])

'''
    BeliefManager Node builder
'''
def build_BeliefManager(namespace, agent_id, init_params):
    debug = (DEBUG_ACTIVE_NODES_PARAM in init_params) and ('belief_manager' in init_params[DEBUG_ACTIVE_NODES_PARAM])
    planning_mode = 'offline'
    if PLANNING_MODE_PARAM in init_params:
        planning_mode = init_params[PLANNING_MODE_PARAM] if init_params[PLANNING_MODE_PARAM] in ['offline', 'online'] else 'offline'

    return Node(
        package='ros2_bdi_core',
        executable='belief_manager',
        name='belief_manager',
        namespace=namespace,
        output='screen',
        parameters= [ {AGENT_ID_PARAM: agent_id}, {DEBUG_PARAM: debug},{PLANNING_MODE_PARAM: planning_mode},  ])
    

'''
    Reactive Rules Event Listener Node builder
'''
def build_EventListener(namespace, agent_id, init_params):
    debug = (DEBUG_ACTIVE_NODES_PARAM in init_params) and ('event_listener' in init_params[DEBUG_ACTIVE_NODES_PARAM])
    planning_mode = 'offline'
    if PLANNING_MODE_PARAM in init_params:
        planning_mode = init_params[PLANNING_MODE_PARAM] if init_params[PLANNING_MODE_PARAM] in ['offline', 'online'] else 'offline'

    return Node(
        package='ros2_bdi_core',
        executable='event_listener',
        name='event_listener',
        namespace=namespace,
        output='screen',
        parameters= [ {AGENT_ID_PARAM: agent_id}, {DEBUG_PARAM: debug},{PLANNING_MODE_PARAM: planning_mode}, ])


'''
    Scheduler Node builder, pass init_params to check, eval and set init parameters for the node
'''
def build_Scheduler(namespace, agent_id, init_params):
    
    debug = (DEBUG_ACTIVE_NODES_PARAM in init_params) and ('scheduler' in init_params[DEBUG_ACTIVE_NODES_PARAM])

    # init. set default values for Scheduler parameters
    reschedule_policy = RESCHEDULE_POLICY_VAL_NO_IF_EXEC
    comp_plan_tries = 16
    exec_plan_tries = 16
    autosubmit_prec = False
    autosubmit_context = False

    # check below for passed values in init

    if RESCHEDULE_POLICY_PARAM in init_params:
        if (init_params[RESCHEDULE_POLICY_PARAM] in [RESCHEDULE_POLICY_VAL_IF_EXEC, RESCHEDULE_POLICY_VAL_IF_EXEC_CLEAN, RESCHEDULE_POLICY_VAL_NO_IF_EXEC] ):
            reschedule_policy = init_params[RESCHEDULE_POLICY_PARAM]
        else:
            log_automatic_set(RESCHEDULE_POLICY_PARAM, reschedule_policy) 

    if COMP_PLAN_TRIES_PARAM in init_params and isinstance(init_params[COMP_PLAN_TRIES_PARAM], int) and init_params[COMP_PLAN_TRIES_PARAM] >= 1:
        comp_plan_tries = init_params[COMP_PLAN_TRIES_PARAM]
    else:
        log_automatic_set(COMP_PLAN_TRIES_PARAM, comp_plan_tries)

    if EXEC_PLAN_TRIES_PARAM in init_params and isinstance(init_params[EXEC_PLAN_TRIES_PARAM], int) and init_params[EXEC_PLAN_TRIES_PARAM] >= 1:
        exec_plan_tries = init_params[EXEC_PLAN_TRIES_PARAM]
    else:
        log_automatic_set(COMP_PLAN_TRIES_PARAM, exec_plan_tries)

    if AUTOSUBMIT_PREC_PARAM in init_params and isinstance(init_params[AUTOSUBMIT_PREC_PARAM], bool):
        autosubmit_prec = init_params[AUTOSUBMIT_PREC_PARAM]
    else:
        log_automatic_set(AUTOSUBMIT_PREC_PARAM, autosubmit_prec)

    if AUTOSUBMIT_CONTEXT_PARAM in init_params and isinstance(init_params[AUTOSUBMIT_CONTEXT_PARAM], bool):
        autosubmit_context = init_params[AUTOSUBMIT_CONTEXT_PARAM]
    else:
        log_automatic_set(AUTOSUBMIT_CONTEXT_PARAM, autosubmit_context)

    planning_mode = 'offline'
    if PLANNING_MODE_PARAM in init_params:
        planning_mode = init_params[PLANNING_MODE_PARAM] if init_params[PLANNING_MODE_PARAM] in ['offline', 'online'] else 'offline'

    interval_search_ms = 500
    if planning_mode == 'online' and SEARCH_INTERVAL_MS_PARAM in init_params and isinstance(init_params[SEARCH_INTERVAL_MS_PARAM], int):
        interval_search_ms = init_params[SEARCH_INTERVAL_MS_PARAM]
        interval_search_ms = interval_search_ms if interval_search_ms >= 100 else 100
    
    max_empty_search_intervals = 16
    if planning_mode == 'online' and MAX_EMPTY_SEARCH_INTERVALS_PARAM in init_params and isinstance(init_params[MAX_EMPTY_SEARCH_INTERVALS_PARAM], int):
        max_empty_search_intervals = init_params[MAX_EMPTY_SEARCH_INTERVALS_PARAM]
        max_empty_search_intervals = max_empty_search_intervals if max_empty_search_intervals > 0 else 1

    return Node(
        package='ros2_bdi_core',
        executable='scheduler_'+planning_mode,
        name='scheduler_'+planning_mode,
        namespace=namespace,
        output='screen',
        parameters=[
            {AGENT_ID_PARAM: agent_id},
            {RESCHEDULE_POLICY_PARAM: reschedule_policy},
            {COMP_PLAN_TRIES_PARAM: comp_plan_tries},
            {EXEC_PLAN_TRIES_PARAM: exec_plan_tries},
            {AUTOSUBMIT_PREC_PARAM: autosubmit_prec},
            {AUTOSUBMIT_CONTEXT_PARAM: autosubmit_context}, 
            {PLANNING_MODE_PARAM: planning_mode},
            {SEARCH_INTERVAL_MS_PARAM: interval_search_ms},
            {MAX_EMPTY_SEARCH_INTERVALS_PARAM: max_empty_search_intervals},
            {DEBUG_PARAM: debug}
        ])


'''
    PlanDirector Node builder, pass init_params to check, eval and set init parameters for the node
'''
def build_PlanDirector(namespace, agent_id, init_params):

    debug = (DEBUG_ACTIVE_NODES_PARAM in init_params) and ('plan_director' in init_params[DEBUG_ACTIVE_NODES_PARAM])

    #  Default init params for Plan Director Node
    abort_surpass_deadline = 2.0

    # validation init param abort after deadline surpassed <n> times during plan execution
    if ABORT_SURPASS_DEADLINE_DEADLINE_PARAM in init_params and (isinstance(init_params[ABORT_SURPASS_DEADLINE_DEADLINE_PARAM], float) or isinstance(init_params[ABORT_SURPASS_DEADLINE_DEADLINE_PARAM], int) and init_params[ABORT_SURPASS_DEADLINE_DEADLINE_PARAM] >= 1):
        abort_surpass_deadline = float(init_params[ABORT_SURPASS_DEADLINE_DEADLINE_PARAM])
    else:
        log_automatic_set(ABORT_SURPASS_DEADLINE_DEADLINE_PARAM, abort_surpass_deadline)

    planning_mode = 'offline'
    if PLANNING_MODE_PARAM in init_params:
        planning_mode = init_params[PLANNING_MODE_PARAM] if init_params[PLANNING_MODE_PARAM] in ['offline', 'online'] else 'offline'

    return Node(
        package='ros2_bdi_core',
        executable='plan_director',
        name='plan_director',
        namespace=namespace,
        output='screen',
        parameters=[
            {AGENT_ID_PARAM: agent_id},
            {ABORT_SURPASS_DEADLINE_DEADLINE_PARAM: abort_surpass_deadline},
            {PLANNING_MODE_PARAM: planning_mode},
            {DEBUG_PARAM: debug}
        ])


'''
    Communication MA Request Handler Node builder, pass init_params to check, eval and set init parameters for the node
    Agent group id is needed too
'''
def build_MARequestHandlerNode(namespace, agent_id, agent_group, init_params):

    debug = (DEBUG_ACTIVE_NODES_PARAM in init_params) and ('ma_request_handler' in init_params[DEBUG_ACTIVE_NODES_PARAM])

    # Default init params for Communication Node
    communication_node_params = [
        {AGENT_ID_PARAM: agent_id},
        {AGENT_GROUP_ID_PARAM: agent_group},
        {DEBUG_PARAM: debug}
    ]

    for BDRW_PARAM in [ACCEPT_BELIEFS_R_PARAM, ACCEPT_BELIEFS_W_PARAM, ACCEPT_DESIRES_R_PARAM, ACCEPT_DESIRES_W_PARAM]:
        if BDRW_PARAM in init_params and is_list_of(init_params[BDRW_PARAM], str) and len(init_params[BDRW_PARAM]) > 0:
            communication_node_params += [{BDRW_PARAM: init_params[BDRW_PARAM]}] 

    # validation init param for accept desires max priorities from [groups] param
    if ACCEPT_DESIRES_MAX_PR_PARAM in init_params and is_list_of(init_params[ACCEPT_DESIRES_MAX_PR_PARAM], float) and len(init_params[ACCEPT_DESIRES_MAX_PR_PARAM]) > 0:
        communication_node_params += [{ACCEPT_DESIRES_MAX_PR_PARAM: init_params[ACCEPT_DESIRES_MAX_PR_PARAM]}] 

    planning_mode = 'offline'
    if PLANNING_MODE_PARAM in init_params:
        planning_mode = init_params[PLANNING_MODE_PARAM] if init_params[PLANNING_MODE_PARAM] in ['offline', 'online'] else 'offline'

    return Node(
        package='ros2_bdi_core',
        executable='ma_request_handler',
        name='ma_request_handler',
        namespace=namespace,
        output='screen',
        parameters=communication_node_params + [{PLANNING_MODE_PARAM: planning_mode},]
    )