"""Simulate n Tello drones"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description(number_of_drones = 7):
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    launch_description = LaunchDescription([
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),
    ])

    for drone_id in range(1, number_of_drones + 1):
        ns = f'drone{drone_id}'
        urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', f'tello_{drone_id}.urdf')

        launch_description.add_action(Node(
            package='tello_gazebo',
            executable='inject_entity.py',
            output='screen',
            arguments=[urdf_path, str(drone_id - 1), '0', '1', '0', f'__ns:={ns}']
        ))

        launch_description.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path, f'__ns:={ns}']
        ))

        launch_description.add_action(Node(
            package='joy',
            executable='joy_node',
            output='screen',
            arguments=[f'__ns:={ns}']
        ))

        launch_description.add_action(Node(
            package='tello_driver',
            executable='tello_joy_main',
            output='screen',
            arguments=[f'__ns:={ns}']
        ))

    return launch_description