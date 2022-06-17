import os
import os.path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Utilities wrapper classes
from bdi_agent_skills import AgentAction
from bdi_agent_skills import AgentSensor

# Utilities functions used in launch script below
from bringup_utils import * 

# Bringup parameters
from bringup_params import *

# Utilities function to build core nodes
from bdi_agent_core import *

'''
    Return LaunchDescription for launch file wrt an agent having the passed params:
        - @agent_id => id of the given agent (mandatory)
        - @agent_group => group id of the given agent (mandatory)
        - @pddl_file => pddl domain file path (mandatory)
        - @init_params => multiple tuples can be defined here (ALL optionals)
            
            ** "init_bset": string file path to YAML file to init the belief set of the agent
            ** "init_dset": string file path to YAML file to init the desire set of the agent

            ** "planning_mode": {"offline", "online"}, default: "offline"
            
            ** "belief_ck": string array of agent groups accepts beliefs CHECK request from
            ** "belief_w": string array of agent groups accepts beliefs WRITE request from
            ** "desire_ck": string array of agent groups accepts desires CHECK request from
            ** "desire_w": string array of agent groups accepts desires WRITE request from
            ** "desire_pr": float array of max priority values [0.0-1.0] accepted by respective agent group
            
            NOTE: "desire_w" and "desire_pr" should have the same length, otherwise
                    communication node put a very low priority to group with no priority related.
           
            ** "rtc_deadline": float value >= 1.0 specifying number of times deadline can be surpassed before aborting plan
            ** "comp_plan_tries": integer value specifying number of times plan can be computed for a desire before discarding it
            ** "exec_plan_tries": integer value specifying number of times plan can be executed for a desire before giving up and discarding it
            
            ** "autosub_prec": boolean value specifying if the agent autosubmit to itself the precondition(s) as desire(s)
                                    if they are not verified and the plan for the given desire cannot be computed due to them
                                    (default value = false)

            ** "autosub_context": boolean value specifying if the agent autosubmit to itself the context condition(s) as desire(s)
                                    if they are not verified and the plan for the given desire cannot be carried on due to them
                                    (default value = false)

            ** "reschedule_policy": string in {"NO_PREEMPT", "PREEMPT"}, otherwise "NO_PREEMPT"
                                    to specify the reschedule policy


            ** "debug_log_active": array containing the nodes of which you want to activate the debug log
'''
def AgentLaunchDescription(
    agent_id='agent0',
    agent_group='group0',
    init_params={},
    actions=[],
    sensors=[],
    run_only_psys2=False
):
    print("Generating launch description for agent \"{}\" in group \"{}\"".format(agent_id, agent_group))
    if not (PDDL_FILE_PARAM in init_params):
        print("Launch description cannot be generated: missing mandatory init_params: \"{}\"".format(PDDL_FILE_PARAM))
        return LaunchDescription()
    
    '''
        [*] GENERAL ENVIRONMENT SETUP
    '''

    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=agent_id,
        description='Namespace definition')
    
    log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value=["info"],
        description='Logging level')
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    
    ld = LaunchDescription()

    #set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(log_level_cmd)

    '''
        PLANNING MODE
    '''
    planning_mode = 'offline'
    if PLANNING_MODE_PARAM in init_params:
        planning_mode = init_params[PLANNING_MODE_PARAM] if init_params[PLANNING_MODE_PARAM] in ['offline', 'online'] else 'offline'

    if planning_mode == 'online': # psys2 won't start its planner (by launch args passed)
        javaff_nodes = Node(
            package='javaff',
            executable='javaff_nodes',
            name='javaff_nodes',
            namespace=namespace,
            output='screen',
            parameters= [
                {"ns": agent_id}
            ]
        )
        
        ld.add_action(javaff_nodes)

    '''
        [*] PLANSYS2 Bringup
    '''
    #Launch PlanSys2 4 core nodes with distributed launch
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_distributed.py')),

        launch_arguments={
            'model_file': init_params[PDDL_FILE_PARAM],
            'namespace': namespace,
            'planning_mode': planning_mode
            }.items()
    )
    # Declare the launch options
    ld.add_action(plansys2_cmd)

    

    if(not run_only_psys2):

        # create tmp folder, delete if already there
        create_tmp_folder_agent(agent_id, True)
        if INIT_BSET_PARAM in init_params:
            # if passed as a param, put init belief set file in the agent tmp folder
            load_init_file(init_params[INIT_BSET_PARAM], 'init_bset.yaml', agent_id)

        if INIT_DSET_PARAM in init_params:
            # if passed as a param, put init desire set file in the agent tmp folder
            load_init_file(init_params[INIT_DSET_PARAM], 'init_dset.yaml', agent_id)   
            
        if INIT_RRULESSET_PARAM in init_params:
            # if passed as a param, put init reactive rules set file in the agent tmp folder
            load_init_file(init_params[INIT_RRULESSET_PARAM], 'init_reactive_rules.yaml', agent_id)   
    
        '''
            [*] PLANSYS MONITOR NODE init.
        '''
        plansys_monitor = build_PlanSysMonitor(namespace, agent_id, init_params)


        '''
            [*] BELIEF MANAGER NODE init.
        '''
        belief_manager = build_BeliefManager(namespace, agent_id, init_params) 
        
        '''
            [*] SCHEDULER NODE init.
        '''
        #  Default init params for Scheduler Node
        scheduler = build_Scheduler(namespace, agent_id, init_params)

        '''
            [*] PLAN DIRECTOR NODE init.
        '''
        plan_director = build_PlanDirector(namespace, agent_id, init_params)
        
        '''
            [*] COMMUNICATION Multi Agent MA Request Handler NODE init.
        '''
        ma_request_handler = build_MARequestHandlerNode(namespace, agent_id, agent_group, init_params)
        
        '''
            [*] EVENT LISTENER NODE init.
        '''
        event_listener = build_EventListener(namespace, agent_id, init_params)
        
        '''
            [*] ADD ROS2_BDI CORE nodes + action(s) & sensor(s) node(s)
        '''
        # Declare plansys2 monitor node
        ld.add_action(plansys_monitor)
        #Add belief manager
        ld.add_action(belief_manager)
        #Add BDI scheduler
        ld.add_action(scheduler)
        #Add plan director
        ld.add_action(plan_director)
        #Add communication manager node
        ld.add_action(ma_request_handler)
        #Add event listener node
        ld.add_action(event_listener)
        
        for act in sensors:
            if isinstance(act, AgentSensor):
                ld.add_action( act.to_node(namespace, [{AGENT_ID_PARAM: agent_id}, {AGENT_GROUP_ID_PARAM: agent_group}]) )

    for act in actions:
        if isinstance(act, AgentAction):
            ld.add_action( act.to_node(namespace, [{AGENT_ID_PARAM: agent_id}, {AGENT_GROUP_ID_PARAM: agent_group}]) )

    return ld
