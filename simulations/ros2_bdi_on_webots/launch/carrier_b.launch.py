import sys
import os
import os.path

from ament_index_python.packages import get_package_share_directory

ros2_bdi_bringup_dir = get_package_share_directory('ros2_bdi_bringup')
sys.path.append(ros2_bdi_bringup_dir + '/launch/')
from bdi_agent import AgentLaunchDescription
from bdi_agent_skills import AgentAction
from bdi_agent_skills import AgentSensor
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    CARRIER_B_AGENT_ID = 'carrier_b'
    CARRIER_B_AGENT_GROUP_ID = 'carriers'

    bdi_onwebots_share_dir = get_package_share_directory('ros2_bdi_on_webots')
    
    # move gripper action
    carrier_b_move = AgentAction(
        package='ros2_bdi_on_webots',
        executable='carrier_move',
        name=CARRIER_B_AGENT_ID+'_move',
        specific_params=[
            {"robot_name": CARRIER_B_AGENT_ID}
        ]
    )
    
    # waypoint sensor 
    carrier_b_move_sensor = AgentSensor(
        package='ros2_bdi_on_webots',
        executable='carrier_move_sensor',
        name=CARRIER_B_AGENT_ID+'_carrier_move_sensor',
        specific_params=[
            {"init_sleep": 1}
        ])


    carrier_b_agent_ld = AgentLaunchDescription(
        agent_id=CARRIER_B_AGENT_ID,
        agent_group=CARRIER_B_AGENT_GROUP_ID,
        init_params={
            'pddl_file': os.path.join(bdi_onwebots_share_dir, 'pddl', 'carrier', 'carrier-domain.pddl'),
            'init_bset': os.path.join(bdi_onwebots_share_dir, 'launch', 'carrier_b_init', 'init_bset_carrier_b.yaml'),
            'init_dset': os.path.join(bdi_onwebots_share_dir, 'launch', 'carrier_b_init', 'init_dset_carrier_b.yaml'),
        },
        actions=[carrier_b_move],
        sensors=[carrier_b_move_sensor]
    ) 

    return carrier_b_agent_ld