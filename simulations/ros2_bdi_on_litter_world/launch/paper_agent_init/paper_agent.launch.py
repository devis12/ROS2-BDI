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
    PAPER_AGENT_ID = 'paper_agent'
    RECYCLER_AGENTS_GROUP_ID = 'recyclers'

    bdi_on_litterworld_share_dir = get_package_share_directory('ros2_bdi_on_litter_world')

    # get static map info status sensor 
    load_map_sensor = AgentSensor(
        package='ros2_bdi_on_litter_world',
        executable='load_map_sensor',
        name=PAPER_AGENT_ID+'_load_map',
        specific_params=[
            {"init_sleep": 2},
            {"sensing_freq": 1.0}
        ])

    # get dynamic map info in a restricted grid 
    agent_area_sensor = AgentSensor(
        package='ros2_bdi_on_litter_world',
        executable='agent_area_sensor',
        name=PAPER_AGENT_ID+'_area_sensor',
        specific_params=[
            {"init_sleep": 4},
            {"sensing_freq": 1.0},
            {"detection_depth": 2}
        ])

    paper_agent_ld = AgentLaunchDescription(
        agent_id=PAPER_AGENT_ID,
        agent_group=RECYCLER_AGENTS_GROUP_ID,
        init_params={
            'pddl_file': os.path.join(bdi_on_litterworld_share_dir, 'pddl', 'recycling-agent-domain.pddl'),
            #'init_bset': os.path.join(bdi_onwebots_share_dir, 'launch', 'gripper_a_init', 'init_bset_gripper_a.yaml'),
            #'init_dset': os.path.join(bdi_onwebots_share_dir, 'launch', 'gripper_a_init', 'init_dset_gripper_a.yaml'),
            #'init_reactive_rules_set': os.path.join(bdi_onwebots_share_dir, 'launch', 'gripper_a_init', 'init_rrules_gripper_a.yaml'),
            'debug_log_active': ['belief_manager']
        },
        actions=[],
        sensors=[load_map_sensor, agent_area_sensor],
        run_only_psys2=False
    ) 

    return paper_agent_ld