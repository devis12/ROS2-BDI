import sys
from ament_index_python.packages import get_package_share_directory

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

    webots = WebotsLauncher(
        world=bdi_onwebots_share_dir + '/worlds/blocksworld.wbt',
    )

    agent_ld = AgentLaunchDescription(
        agent_id=AGENT_ID,
        agent_group=AGENT_GROUP_ID,
        init_params={
            'pddl_file': bdi_onwebots_share_dir + '/pddl/blocks-domain.pddl',
        },
        actions=[],
        sensors=[]
    ) 

    return LaunchDescription(
        [webots, agent_ld]
    )