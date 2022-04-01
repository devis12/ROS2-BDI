import os
import os.path

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription


def generate_launch_description():
    AGENT_ID = 'block_master'

    bdi_onwebots_share_dir = get_package_share_directory('ros2_bdi_on_webots')

    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace definition')
    
    log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value=["info"],
        description='Logging level')
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

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
            'model_file': os.path.join(bdi_onwebots_share_dir, 'pddl', 'blocks-domain.pddl'),
            'namespace': namespace
            }.items()
    )

    bdi_onwebots_share_dir = get_package_share_directory('ros2_bdi_on_webots')
    
    #Launch description for gantry robot simulation on webots with specifically designed ROS2 topics to interact with it
    webots_gauntry_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('webots_ros2_simulations'),
            'launch',
            'gantry_robot.launch.py')),
    )

    # move gripper action
    raw_move_gripper = Node(
        package = 'ros2_bdi_on_webots',
        executable = 'raw_gripper_move',
        name = 'raw_gripper_move',
        namespace = namespace,
        output='screen',
        parameters = []
    )


    # gripper pickup action
    raw_gripper_pickup = Node(
        package = 'ros2_bdi_on_webots',
        executable = 'raw_gripper_pickup',
        name = 'raw_gripper_pickup',
        namespace = namespace,
        output='screen',
        parameters = []
    )

    # gripper putdown action
    raw_gripper_putdown = Node(
        package = 'ros2_bdi_on_webots',
        executable = 'raw_gripper_putdown',
        name = 'raw_gripper_putdown',
        namespace = namespace,
        output='screen',
        parameters = []
    )

    return LaunchDescription([
            stdout_linebuf_envvar,
            declare_namespace_cmd,
            log_level_cmd,
            plansys2_cmd,
            # webots_gauntry_sim,
            raw_move_gripper,
            raw_gripper_pickup,
            raw_gripper_putdown
        ]
    )