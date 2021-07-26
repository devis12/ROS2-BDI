import os
import os.path
import shutil
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def readFile(filepath):
    file_string = ""
    with open(filepath) as f:
        content = f.read().splitlines()
    
    for line in content:
        file_string += line + "\n"
    return file_string



def loadInitFile(pathsource, agent_id):
    # create tmp folder for agent if it does not exist yet
    if os.path.exists('/tmp/'+agent_id):
        shutil.rmtree('/tmp/'+agent_id)
    
    os.mkdir('/tmp/'+agent_id)

    # load init belief set file in /tmp/{agent_id}/init_bset.yaml
    init_bset_filename = '/init_'+agent_id+'_bset.yaml'
    
    if os.path.exists(pathsource + init_bset_filename):
        shutil.copyfile(pathsource + init_bset_filename, '/tmp/'+agent_id+'/init_bset.yaml')
        print(pathsource + init_bset_filename + ' copied into ' + '/tmp/'+agent_id+'/init_bset.yaml')
    else:
        print(pathsource + init_bset_filename + ' invalid path for init. belief set file')

    # load init desire set file in /tmp/{agent_id}/init_dset.yaml
    
    init_dset_filename = '/init_'+agent_id+'_dset.yaml'

    if os.path.exists(pathsource + init_dset_filename):
        shutil.copyfile(pathsource + init_dset_filename, '/tmp/'+agent_id+'/init_dset.yaml')
        print(pathsource + init_dset_filename + ' copied into ' + '/tmp/'+agent_id+'/init_dset.yaml')
    else:
        print(pathsource + init_dset_filename + ' invalid path for init. desire set file')



def generate_launch_description():
    
    CLEANER_NAME = "cleaner"
    
    bdi_tests_share_dir = get_package_share_directory('ros2_bdi_tests')
    loadInitFile(bdi_tests_share_dir+'/launch/init_cleaner_sweeper', CLEANER_NAME)

    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=CLEANER_NAME,
        description='Namespace definition')
    
    log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value=["debug"],
        description='Logging level')
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_distributed.py')),

        launch_arguments={
            'model_file': bdi_tests_share_dir + '/pddl/cleaner_sweeper/cleaner-domain.pddl',
            'namespace': namespace
            }.items()
    )

    ld = LaunchDescription()

    belief_manager = Node(
        package='ros2_bdi_core',
        executable='belief_manager',
        name='belief_manager',
        namespace=namespace,
        output='screen',
        parameters=[{"agent_id": CLEANER_NAME}])

    scheduler = Node(
        package='ros2_bdi_core',
        executable='scheduler',
        name='scheduler',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": CLEANER_NAME},
            {"tries_desire_discard": 16}
        ])

    plan_director = Node(
        package='ros2_bdi_core',
        executable='plan_director',
        name='plan_director',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": CLEANER_NAME}
        ])
    
    plansys2_monitor = Node(
        package='ros2_bdi_core',
        executable='plansys2_monitor',
        name='plansys2_monitor',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": CLEANER_NAME}
        ])

    action_movetoward = Node(
        package='ros2_bdi_tests',
        executable='movetoward',
        name='movetoward',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": CLEANER_NAME}
        ])

    action_doclean = Node(
        package='ros2_bdi_tests',
        executable='doclean',
        name='doclean',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": CLEANER_NAME}
        ])
    
    action_recharge = Node(
        package='ros2_bdi_tests',
        executable='recharge',
        name='recharge',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": CLEANER_NAME}
        ])

    action_asksweeping = Node(
        package='ros2_bdi_tests',
        executable='asksweeping',
        name='asksweeping',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": CLEANER_NAME},
            {"sweeper_id": "sweeper"}
        ])
    
    action_waitsweeping = Node(
        package='ros2_bdi_tests',
        executable='waitsweeping',
        name='waitsweeping',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": CLEANER_NAME},
            {"sweeper_id": "sweeper"}
        ])


    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(log_level_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    # Declare plansys2 monitor node
    ld.add_action(plansys2_monitor)

    #Add belief manager
    ld.add_action(belief_manager)
    #Add BDI scheduler
    ld.add_action(scheduler)
    #Add plan director
    ld.add_action(plan_director)

    #Action performers for agent
    ld.add_action(action_movetoward)
    ld.add_action(action_doclean)
    ld.add_action(action_recharge)
    ld.add_action(action_asksweeping)
    ld.add_action(action_waitsweeping)

    return ld
