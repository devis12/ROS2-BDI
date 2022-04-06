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
    GANTRY_AGENT_ID = 'gantry'
    GANTRY_AGENT_GROUP_ID = 'grippers'

    bdi_onwebots_share_dir = get_package_share_directory('ros2_bdi_on_webots')
    
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
    
    # gripper put_on_carrier action
    gripper_put_on_carrier = AgentAction(
        package='ros2_bdi_on_webots',
        executable='gripper_put_on_carrier',
        name='gripper_put_on_carrier'
    )
    
    # req carrier to come action
    req_carrier_to_come = AgentAction(
        package='ros2_bdi_on_webots',
        executable='req_carrier_to_come',
        name='req_carrier_to_come'
    )
    
    # req carrier to go action
    req_carrier_to_go = AgentAction(
        package='ros2_bdi_on_webots',
        executable='req_carrier_to_go',
        name='req_carrier_to_go'
    )

    gantry_agent_ld = AgentLaunchDescription(
        agent_id=GANTRY_AGENT_ID,
        agent_group=GANTRY_AGENT_GROUP_ID,
        init_params={
            'pddl_file': os.path.join(bdi_onwebots_share_dir, 'pddl', 'gantry', 'gantry-domain.pddl'),
            'init_bset': os.path.join(bdi_onwebots_share_dir, 'launch', 'gantry_init', 'init_bset_gantry.yaml'),
            'init_dset': os.path.join(bdi_onwebots_share_dir, 'launch', 'gantry_init', 'init_dset_gantry.yaml'),
        },
        actions=[move_gripper, gripper_pickup, gripper_putdown, gripper_put_on_carrier, req_carrier_to_come, req_carrier_to_go],
        sensors=[],
        run_only_psys2=False
    ) 

    return gantry_agent_ld