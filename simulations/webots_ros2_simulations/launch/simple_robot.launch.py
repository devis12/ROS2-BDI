import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_simulations')
    robot1_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_simple_robot1.urdf')).read_text()
    robot2_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_simple_robot2.urdf')).read_text()
    
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'simple_world.wbt')
    )

    my_robot1_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot1_description},
        ]
    )
    
    my_robot2_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot2_description},
        ],
        arguments=[
            '--webots-robot-name', 'my_simple_robot2',
            '--webots-node-name', 'my_simple_robot2_driver'
        ]
    )

    return LaunchDescription([
        webots,
        my_robot1_driver,
        my_robot2_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])