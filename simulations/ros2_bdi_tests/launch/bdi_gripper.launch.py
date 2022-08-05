import sys
from ament_index_python.packages import get_package_share_directory

ros2_bdi_bringup_dir = get_package_share_directory('ros2_bdi_bringup')
sys.path.append(ros2_bdi_bringup_dir + '/launch/')
from bdi_agent import AgentLaunchDescription
from bdi_agent import AgentAction

def generate_launch_description():
    
    GRIPPER_NAME = "gripper_a"
    GRIPPER_GROUP_NAME = "grippers"

    bdi_tests_share_dir = get_package_share_directory('ros2_bdi_tests')

    # perform moving
    gripper_move = AgentAction(
        package='ros2_bdi_tests',
        executable='gripper_move_bdi',
        name='gripper_move'
    )

    # perform gripper pickup action
    gripper_pickup = AgentAction(
        package='ros2_bdi_tests',
        executable='gripper_pickup_bdi',
        name='gripper_pickup'
    )
    
    # perform gripper putdown action
    gripper_putdown = AgentAction(
        package='ros2_bdi_tests',
        executable='gripper_putdown_bdi',
        name='gripper_putdown'
    )

    # perform gripper_put_on_carrier action
    gripper_put_on_carrier = AgentAction(
        package='ros2_bdi_tests',
        executable='gripper_put_on_carrier_bdi',
        name='gripper_put_on_carrier'
    )

    # perform req_carrier_to_come action
    req_carrier_to_come1 = AgentAction(
        package='ros2_bdi_tests',
        executable='req_carrier_to_come_bdi',
        name='req_carrier_to_come1'
    )

    # perform req_carrier_to_come action
    req_carrier_to_come2 = AgentAction(
        package='ros2_bdi_tests',
        executable='req_carrier_to_come_bdi',
        name='req_carrier_to_come2'
    )

    ld = AgentLaunchDescription(
        agent_id=GRIPPER_NAME,
        agent_group=GRIPPER_GROUP_NAME,
        init_params={
            'pddl_file': bdi_tests_share_dir + '/pddl/gripper/gripper-domain.pddl',
            'init_bset': bdi_tests_share_dir + '/launch/init_gripper/init_gripper_bset.yaml',
            'init_dset': bdi_tests_share_dir + '/launch/init_gripper/init_gripper_dset.yaml',
            'belief_ck': [],   
            'belief_w':  [],   
            'desire_ck': [],   
            'desire_w':  [],   
            'desire_pr': [],
            'comp_plan_tries': 16,
            'exec_plan_tries': 4,
            'planning_mode':'online',
            'search_interval': 350,
            'debug_log_active': ['javaff', 'scheduler', 'plan_director'],

        },
        actions=[gripper_move, gripper_pickup, gripper_putdown, gripper_put_on_carrier, req_carrier_to_come1, req_carrier_to_come2],
        run_only_psys2=False
    ) 

    return ld