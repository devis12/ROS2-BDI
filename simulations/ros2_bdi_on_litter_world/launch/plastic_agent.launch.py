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
    PLASTIC_AGENT_ID = 'plastic_agent'
    RECYCLER_AGENTS_GROUP_ID = 'recyclers'

    bdi_on_litterworld_share_dir = get_package_share_directory('ros2_bdi_on_litter_world')

    # plastic_agent move action
    plastic_agent_move = AgentAction(
        package='ros2_bdi_on_litter_world',
        executable='move',
        name=PLASTIC_AGENT_ID+'_move',
        specific_params=[]
    )
    
    # plastic_agent pickup action
    plastic_agent_pickup = AgentAction(
        package='ros2_bdi_on_litter_world',
        executable='litter_pickup',
        name=PLASTIC_AGENT_ID+'litter_pickup',
        specific_params=[]
    )

    # plastic_agent recycle action
    plastic_agent_recycle = AgentAction(
        package='ros2_bdi_on_litter_world',
        executable='recycle_plastic',
        name=PLASTIC_AGENT_ID+'recycle_plastic',
        specific_params=[]
    )

    # get static map info status sensor 
    load_map_sensor = AgentSensor(
        package='ros2_bdi_on_litter_world',
        executable='load_map_sensor',
        name=PLASTIC_AGENT_ID+'_load_map',
        specific_params=[
            {"init_sleep": 2},
            {"sensing_freq": 1.0}
        ])

    # get dynamic map info in a restricted grid 
    agent_area_sensor = AgentSensor(
        package='ros2_bdi_on_litter_world',
        executable='agent_area_sensor',
        name=PLASTIC_AGENT_ID+'_area_sensor',
        specific_params=[
            {"init_sleep": 4},
            {"sensing_freq": 1.0},
            {"detection_depth": 2}
        ])

    plastic_agent_ld = AgentLaunchDescription(
        agent_id=PLASTIC_AGENT_ID,
        agent_group=RECYCLER_AGENTS_GROUP_ID,
        init_params={
            'pddl_file': os.path.join(bdi_on_litterworld_share_dir, 'pddl', 'recycling-agent-domain.pddl'),
            'init_bset': os.path.join(bdi_on_litterworld_share_dir, 'launch', 'plastic_agent_init', 'init_bset.yaml'),
            'init_reactive_rules_set': os.path.join(bdi_on_litterworld_share_dir, 'launch', 'plastic_agent_init', 'init_rrules.yaml'),
            'comp_plan_tries': 2,
            'exec_plan_tries': 4,
            'planning_mode':'offline',
            'reschedule_policy': 'NO_PREEMPT',
            'search_interval': 400,
            'min_commit_steps': 1,
            'debug_log_active': ['scheduler']
        },
        actions=[plastic_agent_move, plastic_agent_pickup, plastic_agent_recycle],
        sensors=[load_map_sensor, agent_area_sensor],
        run_only_psys2=False
    ) 

    return plastic_agent_ld