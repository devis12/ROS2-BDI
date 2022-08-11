import sys
from ament_index_python.packages import get_package_share_directory

ros2_bdi_bringup_dir = get_package_share_directory('ros2_bdi_bringup')
sys.path.append(ros2_bdi_bringup_dir + '/launch/')
from bdi_agent import AgentLaunchDescription
from bdi_agent import AgentAction
from bdi_agent import AgentSensor

def generate_launch_description():
    
    PRINTING_ROBOT_NAME = "r2"
    PRINTING_GROUP_NAME = "printers"

    bdi_tests_share_dir = get_package_share_directory('ros2_bdi_tests')

    # perform moving toward
    pf_action_move = AgentAction(
        package='ros2_bdi_tests',
        executable='pf_move',
        name='pf_move'
    )

    # perform docking action
    pf_action_docking = AgentAction(
        package='ros2_bdi_tests',
        executable='pf_docking',
        name='pf_docking'
    )
    
    # printing action
    pf_action_printing = AgentAction(
        package='ros2_bdi_tests',
        executable='pf_printing',
        name='pf_printing'
    )

    # perform docking action
    pf_action_undocking = AgentAction(
        package='ros2_bdi_tests',
        executable='pf_undocking',
        name='pf_undocking'
    )

    # perform recharge action
    pf_action_recharge = AgentAction(
        package='ros2_bdi_tests',
        executable='pf_recharge',
        name='pf_recharge'
    )
    
    # unload_printed_docs action
    pf_action_unload_printed_docs = AgentAction(
        package='ros2_bdi_tests',
        executable='pf_unload_printed_docs',
        name='pf_unload_printed_docs'
    )

    # printers availability sensor 
    # printers_availability = AgentSensor(
    #     package='ros2_bdi_tests',
    #     executable='printers_sensor',
    #     name='printers_sensor',
    #     specific_params=[
    #         {"init_sleep": 2},
    #         {"sensing_freq": 2.0},
    #         {"debug": True}
    #     ])

    ld = AgentLaunchDescription(
        agent_id=PRINTING_ROBOT_NAME,
        agent_group=PRINTING_GROUP_NAME,
        init_params={
            'pddl_file': bdi_tests_share_dir + '/pddl/printing-floor/printing-domain.pddl',
            'init_bset': bdi_tests_share_dir + '/launch/init_printing_floor/init_bset.yaml',
            'init_dset': bdi_tests_share_dir + '/launch/init_printing_floor/init_dset.yaml',
            'init_reactive_rules_set': bdi_tests_share_dir + '/launch/init_printing_floor/init_rrules.yaml',
            'comp_plan_tries': 2,
            'exec_plan_tries': 4,
            'planning_mode':'online',
            'search_interval': 150,
            'debug_log_active': []
        },
        actions=[pf_action_move, pf_action_docking, pf_action_printing, pf_action_undocking, pf_action_recharge, pf_action_unload_printed_docs],
        sensors=[],
        run_only_psys2=False
    ) 

    return ld