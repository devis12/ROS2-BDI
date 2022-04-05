import os
import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

def generate_launch_description():

    bdi_onwebots_share_dir = get_package_share_directory('ros2_bdi_on_webots')
    
    #Launch description for gantry robot simulation on webots with specifically designed ROS2 topics to interact with it
    webots_gauntry_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('webots_ros2_simulations'),
            'launch',
            'blocks_world.launch.py')),
    )

    # Launch description for gantry agent
    gantry_agent_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bdi_onwebots_share_dir,
            'launch',
            'gantry.launch.py')),
    )
    
    # # Launch description for carrier_a agent
    # carrier_a_agent_ld = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         bdi_onwebots_share_dir,
    #         'launch',
    #         'carrier_a.launch.py')),
    # )
    
    # # Launch description for carrier_b agent
    # carrier_b_agent_ld = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         bdi_onwebots_share_dir,
    #         'launch',
    #         'carrier_b.launch.py')),
    # )
    
    # # Launch description for carrier_c agent
    # carrier_c_agent_ld = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         bdi_onwebots_share_dir,
    #         'launch',
    #         'carrier_c.launch.py')),
    # )

    return LaunchDescription([
            webots_gauntry_sim,
            # gantry_agent_ld,
            # carrier_a_agent_ld,
            # carrier_b_agent_ld,
            # carrier_c_agent_ld,
        ]
    )