"""
# Aerial Robotics UTU 2024: Group 5 - Open Project
 
## Description:
This code is meant to take off 3 tello drones, then 
follow each other while forming a triangle, in a 
gazebo simulation, using ROS2 and OpenCV.

Based on: https://github.com/TIERS/drone_racing_ros2/tree/main

## Installation:
Run this on your console once: (or VisualStudio Code terminal, if you rather)

    mkdir -p ~/drone_racing_ros2_ws/src
    cd ~/drone_racing_ros2_ws/src
    git clone https://github.com/TIERS/drone_racing_ros2.git
    cd ..
    source /opt/ros/galactic/setup.bash
    colcon build

## Usage
Run this every time you want to run the gazebo environment

    cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    #ros2 launch tello_gazebo simple_launch.py

Then, run this file with python from console and you are good to go :D

## Extra
You can also add the path to to your ".bashrc" file, if you want to run the python file faster...

    nano ~/.bashrc

And then add:

    # Drone_Racing_ROS2
    cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    export GAZEBO_MODEL_PATH=/home/julian/drone_racing_ros2_ws/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh

Enjoy!
 
## Authors: 

     - Prashan Herath [prashan.r.herathmudiyanselage@utu.fi]
     - Julian C. Paez P. [julian.c.paezpineros@utu.fi]
     - Michalis Iona [michalis.l.iona@utu.fi]

For: University of Turku - TIERS 
Course: Aerial Robotics and Multi-Robot Systems 
Date: May 16th, 2024 

"""




import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import subprocess

class DroneController(Node):
    def __init__(self, takeoff_delay=2):
        super().__init__('drone_controller')
        self.takeoff_delay = takeoff_delay  # Time in seconds between each drone's takeoff
        
        # Drone namespaces
        self.drone_namespaces = ['drone1', 'drone2', 'drone3']
        self.action_clients = {}
        self.velocity_publishers = {}
        self.odom_subscribers = {}
        
        for ns in self.drone_namespaces:
            # Create client for sending takeoff and land commands to each drone
            self.action_clients[ns] = self.create_client(TelloAction, f'/{ns}/tello_action')
            
            # Create publisher for sending Twist messages to each drone
            self.velocity_publishers[ns] = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
            
            # Create subscriber for receiving odometry information from each drone
            self.odom_subscribers[ns] = self.create_subscription(Odometry, f'/{ns}/odom', self.odom_callback_factory(ns), 10)
            
            # Wait for the service to become available
            while not self.action_clients[ns].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {ns}/tello_action not available yet, waiting...')
        
        self.positions = {ns: None for ns in self.drone_namespaces}
        self.last_drone2_pos = None
        
        # Take off each drone sequentially with a delay
        self.takeoff_sequentially()

    def takeoff_sequentially(self):
        for ns in self.drone_namespaces:
            req = TelloAction.Request()
            req.cmd = 'takeoff'
            self.get_logger().info(f'Attempting to take off {ns}...')
            future = self.action_clients[ns].call_async(req)
            
            # Wait for the future to complete
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                self.get_logger().info(f'{ns} has taken off.')
            else:
                self.get_logger().error(f'Exception while calling service: {future.exception()}')
                
            time.sleep(self.takeoff_delay)  # Use the class variable for the delay

        # Move drones 1 and 3 to maintain a triangular formation
        self.move_drones_to_formation()

        # Start a timer to log the positions of the drones
        self.create_timer(2.0, self.log_positions)

    def odom_callback_factory(self, drone_ns):
        def odom_callback(msg):
            position = msg.pose.pose.position
            self.positions[drone_ns] = (position.x, position.y, position.z)

            #print("  x:", msg.point.x)
            
            # Track movements of Drone 2
            if drone_ns == 'drone2':
                self.adjust_followers(position)
        
        return odom_callback

    def log_positions(self):
        for ns in self.drone_namespaces:
            pos = self.positions[ns]
            if pos:
                self.get_logger().info(f'Position of {ns}: x={pos[0]}, y={pos[1]}, z={pos[2]}')
            else:
                self.get_logger().info(f'Position of {ns} not yet available.')

    def move_drones_to_formation(self):
        twist_msg1 = Twist()
        twist_msg3 = Twist()

        # Move Drone 1 and Drone 3 in the positive y-direction
        twist_msg1.linear.y = 0.5
        twist_msg3.linear.y = 0.5

        # Publish the move commands
        self.velocity_publishers['drone1'].publish(twist_msg1)
        self.velocity_publishers['drone3'].publish(twist_msg3)

        # Sleep for half a second to give time for the movement
        time.sleep(0.5)

        # Stop the movement
        twist_msg1.linear.y = 0.0
        twist_msg3.linear.y = 0.0

        self.velocity_publishers['drone1'].publish(twist_msg1)
        self.velocity_publishers['drone3'].publish(twist_msg3)

    def adjust_followers(self, position):
        if self.last_drone2_pos is not None:
            delta_x = position.x - self.last_drone2_pos.x
            delta_y = position.y - self.last_drone2_pos.y
            delta_z = position.z - self.last_drone2_pos.z

            # Proportional gain for follower movement
            gain = 1.0

            twist_msg1 = Twist()
            twist_msg3 = Twist()

            # Drones 1 and 3 adjust based on Drone 2's movement
            twist_msg1.linear.x = gain * delta_x
            twist_msg1.linear.y = gain * delta_y
            twist_msg1.linear.z = gain * delta_z

            twist_msg3.linear.x = gain * delta_x
            twist_msg3.linear.y = gain * delta_y
            twist_msg3.linear.z = gain * delta_z

            # Publish the move commands
            self.velocity_publishers['drone1'].publish(twist_msg1)
            self.velocity_publishers['drone3'].publish(twist_msg3)

        # Update last known position of Drone 2
        self.last_drone2_pos = position

def launch_gazebo():
    subprocess.Popen(['ros2', 'launch', 'tello_gazebo', 'openproject.py'])

def main(args=None):
    # Launch the Gazebo environment
    launch_gazebo()

    # Wait a bit to ensure Gazebo is up and running
    time.sleep(6)

    # Initialize the ROS2 client library
    rclpy.init(args=args)

    # Instantiate DroneController with desired takeoff delay
    drone_controller = DroneController(takeoff_delay=5)  # Change the delay here
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()