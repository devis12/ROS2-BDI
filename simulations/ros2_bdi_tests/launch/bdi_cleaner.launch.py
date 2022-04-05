import sys
from ament_index_python.packages import get_package_share_directory

ros2_bdi_bringup_dir = get_package_share_directory('ros2_bdi_bringup')
sys.path.append(ros2_bdi_bringup_dir + '/launch/')
from bdi_agent import AgentLaunchDescription
from bdi_agent import AgentAction

def generate_launch_description():
    
    CLEANER_NAME = "cleaner"
    CLEANER_GROUP_NAME = "cleaners"
    SWEEPER_GROUP_NAME = "sweepers"

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

    # asksweeping actions
    action_asksweeping = []
    for i in range(0, 3):
        action_asksweeping.append( 
            AgentAction(
                package='ros2_bdi_tests',
                executable='asksweeping_bdi',
                name='asksweeping_{}'.format(i)
            )
        )

    ld = AgentLaunchDescription(
        agent_id=CLEANER_NAME,
        agent_group=CLEANER_GROUP_NAME,
        init_params={
            'pddl_file': bdi_tests_share_dir + '/pddl/cleaner_sweeper/cleaner-domain.pddl',
            'init_bset': bdi_tests_share_dir + '/launch/init_cleaner_sweeper/init_cleaner_bset.yaml',
            'init_dset': bdi_tests_share_dir + '/launch/init_cleaner_sweeper/init_cleaner_dset.yaml',
            'belief_ck': [CLEANER_GROUP_NAME, SWEEPER_GROUP_NAME],   
            'belief_w':  [CLEANER_GROUP_NAME, SWEEPER_GROUP_NAME],   
            'desire_ck': [CLEANER_GROUP_NAME, SWEEPER_GROUP_NAME],   
            'desire_w':  [CLEANER_GROUP_NAME, SWEEPER_GROUP_NAME],   
            'desire_pr': [0.8, 0.6],
            'comp_plan_tries': 16,
            'exec_plan_tries': 4
        },
        actions=[action_movetoward, action_doclean, action_recharge]+action_asksweeping
    ) 

    return ld