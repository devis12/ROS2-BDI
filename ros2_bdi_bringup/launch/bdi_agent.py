import os
import os.path
import shutil
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

class AgentAddons:

    def __init__(self, package = '', executable = '', name = '', specific_params = []): # constructor
        self.package = package # action name
        self.executable = executable # executable name
        self.name = name # node name
        self.specific_params =  specific_params # specific parameters (in addition to agent_id, group and co.)
    
    def to_node(self, namespace = '', base_params = []):
        return Node(
            package = self.package,
            executable = self.executable,
            name = self.name,
            namespace = namespace,
            output='screen',
            parameters = base_params + self.specific_params 
        )

class AgentAction(AgentAddons):
    def __init__(self, package = '', executable = '', name = '', specific_params = []):
        super().__init__(package, executable, name, specific_params)
    
class AgentSensor(AgentAddons):
    def __init__(self, package = '', executable = '', name = '', specific_params = []):
        super().__init__(package, executable, name, specific_params)


'''
#read a file into a string (debugging purposes)
def readFile(filepath):
    file_string = ""
    with open(filepath) as f:
        content = f.read().splitlines()
    
    for line in content:
        file_string += line + "\n"
    return file_string
'''

'''
    Create tmp folder for agent having id @agent_id
    Wipe content if already exists and @wipe equals True
'''
def create_tmp_folder_agent(agent_id, wipe = True):
    # remove tmp folder for agent if it does exist already
    if wipe and os.path.exists('/tmp/'+agent_id):
        shutil.rmtree('/tmp/'+agent_id)

    if not os.path.exists('/tmp/'+agent_id):
        # create fresh new tmp folder for agent
        os.mkdir('/tmp/'+agent_id)

def load_init_file(pathsource, init_filename, agent_id):
    if os.path.exists(pathsource):
        shutil.copyfile(pathsource, '/tmp/'+agent_id+'/'+init_filename)
    else:
        print(pathsource + '\t invalid path for init. belief/desire set file')


def log_automatic_set(param, param_value):
    print('Invalid or unspecified value for \"{}\" which will be automatically set to {}'.format(param, param_value))

def is_list_of(l, type):
    if not isinstance(l, list):
            return False
    else:
        for el in l:
            if not isinstance(el, type):
                return False
        return True

AGENT_ID_PARAM = 'agent_id'
AGENT_GROUP_ID_PARAM = 'agent_group'
PDDL_FILE_PARAM = 'pddl_file'

INIT_BSET_PARAM = 'init_bset'
INIT_DSET_PARAM = 'init_dset'

ACCEPT_BELIEFS_PARAM = 'accept_beliefs_from'
ACCEPT_DESIRES_PARAM = 'accept_desires_from'
ACCEPT_DESIRES_MAX_PR_PARAM = 'accept_desires_max_priorities'

ABORT_AFTER_DEADLINE_PARAM = 'abort_after_deadline'
COMP_PLAN_TRIES_PARAM = 'compute_plan_tries'
EXEC_PLAN_TRIES_PARAM = 'exec_plan_tries'

RESCHEDULE_POLICY_PARAM = 'reschedule_policy'
RESCHEDULE_POLICY_VAL_NO_IF_EXEC = 'IF_EXEC'
RESCHEDULE_POLICY_VAL_IF_EXEC = 'NO_IF_EXEC'



'''
    Return LaunchDescription for launch file wrt an agent having the passed params:
        - @agent_id => id of the given agent (mandatory)
        - @agent_group => group id of the given agent (mandatory)
        - @pddl_file => pddl domain file path (mandatory)
        - @init_params => multiple tuples can be defined here (ALL optionals)
            
            ** "init_bset": string file path to YAML file to init the belief set of the agent
            ** "init_dset": string file path to YAML file to init the desire set of the agent
            
            ** "accept_beliefs_from": string array of agent groups accepts beliefs request from
            ** "accept_desires_from": string array of agent groups accepts desires request from
            ** "accept_desires_max_priorities": float array of max priority values [0.0-1.0] accepted by respective agent group
            
            NOTE: "accept_desires_from" and "accept_desires_max_priorities" should have the same length, otherwise
                    communication node put a very low priority to group with no priority related.
            NOTE: If not overwritten, by default it's automatically put the name of the current agent group in
                    both "accept_beliefs_from" and "accept_desires_from" (with priority value 0.5)
           
            ** "abort_after_deadline": float value >= 1.0 specifying number of times deadline can be surpassed before aborting plan
            ** "compute_plan_tries": integer value specifying number of times plan can be computed for a desire before discarding it
            ** "exec_plan_tries": integer value specifying number of times plan can be executed for a desire before giving up and discarding it
            
            ** "reschedule_policy": string in {"NO_IF_EXEC", "IF_EXEC"}, otherwise "NO_IF_EXEC"
                                    to specify the reschedule policy
'''
def AgentLaunchDescription(
    agent_id='agent0',
    agent_group='group0',
    init_params={},
    actions=[],
    sensors=[]
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
    
    # create tmp folder, delete if already there
    create_tmp_folder_agent(agent_id, True)
    if INIT_BSET_PARAM in init_params:
        # if passed as a param, put init belief set file in the agent tmp folder
        load_init_file(init_params[INIT_BSET_PARAM], 'init_bset.yaml', agent_id)

    if INIT_DSET_PARAM in init_params:
        # if passed as a param, put init desire set file in the agent tmp folder
        load_init_file(init_params[INIT_DSET_PARAM], 'init_dset.yaml', agent_id)   

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
            'namespace': namespace
            }.items()
    )
    # Declare the launch options
    ld.add_action(plansys2_cmd)
    


    '''
        [*] PLANSTS2 MONITOR NODE init.
    '''

    plansys2_monitor = Node(
        package='ros2_bdi_core',
        executable='plansys2_monitor',
        name='plansys2_monitor',
        namespace=namespace,
        output='screen',
        parameters=[
            {AGENT_ID_PARAM: agent_id}
        ])
    
    '''
        [*] BELIEF MANAGER NODE init.
    '''

    belief_manager = Node(
        package='ros2_bdi_core',
        executable='belief_manager',
        name='belief_manager',
        namespace=namespace,
        output='screen',
        parameters=[{AGENT_ID_PARAM: agent_id}])
    
    '''
        [*] SCHEDULER NODE init.
    '''
    #  Default init params for Scheduler Node
    reschedule_policy = 'NO_IF_EXEC'
    comp_plan_tries = 16
    exec_plan_tries = 16

    if RESCHEDULE_POLICY_PARAM in init_params:
        if (init_params[RESCHEDULE_POLICY_PARAM] == RESCHEDULE_POLICY_VAL_IF_EXEC or init_params[RESCHEDULE_POLICY_PARAM] == RESCHEDULE_POLICY_VAL_NO_IF_EXEC ):
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

    scheduler = Node(
        package='ros2_bdi_core',
        executable='scheduler',
        name='scheduler',
        namespace=namespace,
        output='screen',
        parameters=[
            {AGENT_ID_PARAM: agent_id},
            {RESCHEDULE_POLICY_PARAM: reschedule_policy},
            {COMP_PLAN_TRIES_PARAM: comp_plan_tries},
            {EXEC_PLAN_TRIES_PARAM: exec_plan_tries}
        ])

    '''
        [*] PLAN DIRECTOR NODE init.
    '''
    #  Default init params for Plan Director Node
    abort_after_deadline = 2.0

    # validation init param abort after deadline surpassed <n> times during plan execution
    if ABORT_AFTER_DEADLINE_PARAM in init_params and (isinstance(init_params[ABORT_AFTER_DEADLINE_PARAM], float) or isinstance(init_params[ABORT_AFTER_DEADLINE_PARAM], int)):
        abort_after_deadline = float(init_params[ACCEPT_BELIEFS_PARAM])
    else:
        log_automatic_set(ABORT_AFTER_DEADLINE_PARAM, abort_after_deadline)

    plan_director = Node(
        package='ros2_bdi_core',
        executable='plan_director',
        name='plan_director',
        namespace=namespace,
        output='screen',
        parameters=[
            {AGENT_ID_PARAM: agent_id},
            {ABORT_AFTER_DEADLINE_PARAM: abort_after_deadline}
        ])
    
    '''
        [*] COMMUNICATION NODE init.
    '''
    # Default init params for Communication Node
    communication_node_params = [
        {AGENT_ID_PARAM: agent_id},
        {AGENT_GROUP_ID_PARAM: agent_group}
    ]

    # validation init param for accept beliefs from [groups] param
    if ACCEPT_BELIEFS_PARAM in init_params and is_list_of(init_params[ACCEPT_BELIEFS_PARAM], str) and len(init_params[ACCEPT_BELIEFS_PARAM]) > 0:
       communication_node_params += [{ACCEPT_BELIEFS_PARAM: init_params[ACCEPT_BELIEFS_PARAM]}] 

    # validation init param for accept desires from [groups] param
    if ACCEPT_DESIRES_PARAM in init_params and is_list_of(init_params[ACCEPT_DESIRES_PARAM], str) and len(init_params[ACCEPT_DESIRES_PARAM]) > 0:
        communication_node_params += [{ACCEPT_DESIRES_PARAM: init_params[ACCEPT_DESIRES_PARAM]}] 

    # validation init param for accept desires max priorities from [groups] param
    if ACCEPT_DESIRES_MAX_PR_PARAM in init_params and is_list_of(init_params[ACCEPT_DESIRES_MAX_PR_PARAM], float) and len(init_params[ACCEPT_DESIRES_MAX_PR_PARAM]) > 0:
        communication_node_params += [{ACCEPT_DESIRES_MAX_PR_PARAM: init_params[ACCEPT_DESIRES_MAX_PR_PARAM]}] 

    communications_manager = Node(
        package='ros2_bdi_core',
        executable='communications',
        namespace=namespace,
        output='screen',
        parameters=communication_node_params 
    )
    
    '''
        [*] ADD ROS2_BDI CORE nodes + action(s) & sensor(s) node(s)
    '''
    # Declare plansys2 monitor node
    ld.add_action(plansys2_monitor)
    #Add belief manager
    ld.add_action(belief_manager)
    #Add BDI scheduler
    ld.add_action(scheduler)
    #Add plan director
    ld.add_action(plan_director)
    #Add communication manager node
    ld.add_action(communications_manager)
    
    for act in actions:
        if isinstance(act, AgentAction):
            ld.add_action( act.to_node(namespace, [{AGENT_ID_PARAM: agent_id}, {AGENT_GROUP_ID_PARAM: agent_group}]) )
    
    for act in sensors:
        if isinstance(act, AgentSensor):
            ld.add_action( act.to_node(namespace, [{AGENT_ID_PARAM: agent_id}, {AGENT_GROUP_ID_PARAM: agent_group}]) )

    return ld
