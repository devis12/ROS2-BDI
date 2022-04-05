import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_simulations')
    carrier_a_robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'carrier_a_robot.urdf')).read_text()
    carrier_b_robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'carrier_b_robot.urdf')).read_text()
    carrier_c_robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'carrier_c_robot.urdf')).read_text()
    gantry_robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'gantry_robot.urdf')).read_text()
    
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'blocksworld.wbt')
    )

    gantry_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        name='gantry_driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'gantry'},
        parameters=[
            {'robot_description': gantry_robot_description},
        ]
    )
    
    carrier_a_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        name='carrier_a_driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'carrier_a'},
        parameters=[
            {'robot_description': carrier_a_robot_description},
        ]
    )
    
    carrier_b_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        name='carrier_b_driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'carrier_b'},
        parameters=[
            {'robot_description': carrier_b_robot_description},
        ]
    )
    
    carrier_c_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        name='carrier_c_driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'carrier_c'},
        parameters=[
            {'robot_description': carrier_c_robot_description},
        ]
    )

    return LaunchDescription([
        webots,
        gantry_robot_driver,
        carrier_a_robot_driver,
        carrier_b_robot_driver,
        carrier_c_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])