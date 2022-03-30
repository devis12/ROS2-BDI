import sys
import os
import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

ros2_bdi_bringup_dir = get_package_share_directory('ros2_bdi_bringup')
sys.path.append(ros2_bdi_bringup_dir + '/launch/')
from bdi_agent import AgentLaunchDescription
from bdi_agent_skills import AgentAction
from bdi_agent_skills import AgentSensor
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    AGENT_ID = 'block_master'
    AGENT_GROUP_ID = 'block_masters'

    bdi_onwebots_share_dir = get_package_share_directory('ros2_bdi_on_webots')
    
    #Launch description for gantry robot simulation on webots with specifically designed ROS2 topics to interact with it
    webots_gauntry_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('webots_ros2_simulations'),
            'launch',
            'gantry_robot.launch.py')),
    )

    # move gripper action
    move_gripper = AgentAction(
        package='ros2_bdi_on_webots',
        executable='gripper_move',
        name='gripper_move'
    )

    # gripper pickup action
    gripper_pickup = AgentAction(
        package='ros2_bdi_on_webots',
        executable='gripper_pickup',
        name='gripper_pickup'
    )

    # gripper putdown action
    gripper_putdown = AgentAction(
        package='ros2_bdi_on_webots',
        executable='gripper_putdown',
        name='gripper_putdown'
    )

    agent_ld = AgentLaunchDescription(
        agent_id=AGENT_ID,
        agent_group=AGENT_GROUP_ID,
        init_params={
            'pddl_file': os.path.join(bdi_onwebots_share_dir, 'pddl', 'blocks-domain.pddl'),
            'init_bset': os.path.join(bdi_onwebots_share_dir, 'launch', 'init_bset_blocksworld.yaml'),
            'init_dset': os.path.join(bdi_onwebots_share_dir, 'launch', 'init_dset_blocksworld.yaml'),
        },
        actions=[move_gripper, gripper_pickup, gripper_putdown],
        sensors=[]
    ) 

    return LaunchDescription([
            agent_ld,
            webots_gauntry_sim
        ]
    )