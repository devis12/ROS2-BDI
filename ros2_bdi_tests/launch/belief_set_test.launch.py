import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    AGENT_NAME = "agent1"

    bdi_tests_share_dir = get_package_share_directory('ros2_bdi_tests')
    
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=AGENT_NAME,
        description='Namespace definition')
    
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

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    #Add belief manager
    ld.add_action(belief_manager)

    return ld
