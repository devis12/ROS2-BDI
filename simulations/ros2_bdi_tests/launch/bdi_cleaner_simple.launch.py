import sys
from ament_index_python.packages import get_package_share_directory

ros2_bdi_bringup_dir = get_package_share_directory('ros2_bdi_bringup')
sys.path.append(ros2_bdi_bringup_dir + '/launch/')
from bdi_agent import AgentLaunchDescription
from bdi_agent_skills import AgentAction
from bdi_agent_skills import AgentSensor

def generate_launch_description():
    AGENT_ID = 'cleaner'
    AGENT_GROUP_ID = 'cleaner'

    bdi_tests_share_dir = get_package_share_directory('ros2_bdi_tests')

    # perform moving toward
    action_movetoward = AgentAction(
        package='ros2_bdi_tests',
        executable='movetoward_bdi',
        name='movetoward'
    )

    # perform cleaning action
    action_doclean = AgentAction(
        package='ros2_bdi_tests',
        executable='doclean_bdi',
        name='doclean'
    )
    
    # recharging action
    action_recharge = AgentAction(
        package='ros2_bdi_tests',
        executable='recharge_bdi',
        name='recharge'
    )

    # waypoint sensor 
    wp_sensor = AgentSensor(
        package='ros2_bdi_tests',
        executable='wp_sensor',
        name='wp_sensor',
        specific_params=[
            {"init_sleep": 3},
            {"debug": False}
        ])

    ld = AgentLaunchDescription(
        agent_id=AGENT_ID,
        agent_group=AGENT_GROUP_ID,
        init_params={
            'pddl_file': bdi_tests_share_dir + '/pddl/cleaner_simple/cleaner-domain.pddl',
            'init_bset': bdi_tests_share_dir + '/launch/init_cleaner_simple/init_bset.yaml',
            'init_dset': bdi_tests_share_dir + '/launch/init_cleaner_simple/init_dset.yaml',
            'init_reactive_rules_set': bdi_tests_share_dir + '/launch/init_cleaner_simple/init_rrules.yaml',
            'autosub_prec': True,
            'autosub_cont': True,
            'reschedule_policy': 'NO_PREEMPT',
            'planning_mode': 'offline',
            'debug_log_active': ['belief_manager', 'scheduler', 'plan_director']
        },
        actions=[action_movetoward, action_doclean, action_recharge],
        sensors=[wp_sensor],
        run_only_psys2=False
    ) 

    return ld