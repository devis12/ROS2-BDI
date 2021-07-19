import os
import os.path
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

def generate_launch_description():
    AGENT_NAME = "agent1"

    bdi_tests_share_dir = get_package_share_directory('ros2_bdi_tests')
    
    bdi_core_share_dir = get_package_share_directory('ros2_bdi_core')
    pddl_test_domain = readFile(bdi_core_share_dir + "/pddl/cleaner-domain.pddl")
    pddl_test_problem = readFile(bdi_core_share_dir + "/pddl/cleaner-problem.pddl")

    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=AGENT_NAME,
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
            'model_file': bdi_tests_share_dir + '/pddl/cleaner-domain.pddl',
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
        parameters=[{"agent_id": AGENT_NAME}])

    scheduler = Node(
        package='ros2_bdi_core',
        executable='scheduler',
        name='scheduler',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": AGENT_NAME},
            {"pddl_test_domain": pddl_test_domain},
            {"pddl_test_problem": pddl_test_problem}
        ])

    plan_director = Node(
        package='ros2_bdi_core',
        executable='plan_director',
        name='plan_director',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": AGENT_NAME}
        ])

    action_movetoward = Node(
        package='ros2_bdi_tests',
        executable='movetoward',
        name='movetoward',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": AGENT_NAME}
        ])

    action_doclean = Node(
        package='ros2_bdi_tests',
        executable='doclean',
        name='doclean',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": AGENT_NAME}
        ])
    
    action_recharge = Node(
        package='ros2_bdi_tests',
        executable='recharge',
        name='recharge',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": AGENT_NAME}
        ])

    wp_sensor = Node(
        package='ros2_bdi_tests',
        executable='wp_sensor',
        name='wp_sensor',
        namespace=namespace,
        output='screen',
        parameters=[
            {"agent_id": AGENT_NAME},
            {"init_sleep": 4}
        ])


    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(log_level_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

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

    #Sensors for agent
    ld.add_action(wp_sensor)

    return ld
