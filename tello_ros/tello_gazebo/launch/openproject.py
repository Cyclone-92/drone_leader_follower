"""Simulate n Tello drones"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler


def generate_launch_description(number_of_drones = 7):
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ],
        output='screen',
        name='gazebo'
    )

    launch_description = LaunchDescription([gazebo])

    # Spawn drones
    for drone_id in range(1, number_of_drones + 1):
        ns = f'drone{drone_id}'
        urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', f'tello_{drone_id}.urdf')

        inject_entity = Node(
            package='tello_gazebo',
            executable='inject_entity.py',
            output='screen',
            arguments=[urdf_path, str(drone_id - 1), '0', '1', '0', f'__ns:={ns}'],
            name=f'inject_entity_{ns}'
        )

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path, f'__ns:={ns}'],
            name=f'robot_state_publisher_{ns}'
        )

        joy_node = Node(
            package='joy',
            executable='joy_node',
            output='screen',
            arguments=[f'__ns:={ns}'],
            name=f'joy_node_{ns}'
        )

        tello_joy_main = Node(
            package='tello_driver',
            executable='tello_joy_main',
            output='screen',
            arguments=[f'__ns:={ns}'],
            name=f'tello_joy_main_{ns}'
        )

        launch_description.add_action(inject_entity)
        launch_description.add_action(robot_state_publisher)
        launch_description.add_action(joy_node)
        launch_description.add_action(tello_joy_main)

    # Delay execution of the Python script until after all nodes have started
    python_script = ExecuteProcess(
        cmd=['python3', os.path.join(get_package_share_directory('tello_gazebo'), 'src', 'OpenProject.py')],
        output='screen'
    )

    # Add a timer to wait for some seconds to ensure everything is stable
    delay_action = TimerAction(
        period=5.0,  # Adjust the delay as needed to ensure all nodes are fully started and stable
        actions=[python_script]
    )

    launch_description.add_action(delay_action)

    return launch_description