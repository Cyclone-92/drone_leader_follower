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

from itertools import zip_longest
import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import subprocess
import re
import numpy as np

class DroneController(Node):
    def __init__(self, takeoff_delay=2):
        super().__init__('drone_controller')
        self.takeoff_delay = takeoff_delay  # Time in seconds between each drone's takeoff
        self.policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        result = subprocess.run(['ros2','topic','list'], capture_output=True, text=True)

        drone_values = re.findall(r'drone\d+', result.stdout)

        # get all the drones
        self.drone_namespaces = np.unique(drone_values).tolist()
        # Find leader drone
        self.leader_drone = self.leader(self.drone_namespaces)
        print(self.leader_drone)
        # Find the index of the leader
        index = self.drone_namespaces.index(self.leader_drone)
        # get followers
        self.followers = np.delete(self.drone_namespaces,index)

        self.action_clients = {}
        self.velocity_publishers = {}
        self.odom_subscribers = {}
        self.take_off = False
        
        for ns in self.drone_namespaces:
            # Create client for sending takeoff and land commands to each drone
            self.action_clients[ns] = self.create_client(TelloAction, f'/{ns}/tello_action')
            
            # Create publisher for sending Twist messages to each drone
            self.velocity_publishers[ns] = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
            
            # # Create subscriber for receiving odometry information from each drone
            self.odom_subscribers[ns] = self.create_subscription(Odometry, f'/{ns}/odom', self.odom_callback_factory(ns), self.policy)
            
            # Wait for the service to become available
            while not self.action_clients[ns].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {ns}/tello_action not available yet, waiting...')


        self.positions = {ns: None for ns in self.drone_namespaces}
        self.last_drone2_pos = None
        
        # Take off each drone sequentially with a delay
        self.takeoff_sequentially()


    def leader(self,arr):
        n = len(arr)
        if n % 2 == 1:
            middle_index = n // 2
            return arr[middle_index]
        else:
            middle_index1 = n // 2 - 1
            # middle_index2 = n // 2
            return arr[middle_index1]
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
        self.take_off = True
        # Start a timer to log the positions of the drones
        # self.create_timer(2.0, self.log_positions)

    def odom_callback_factory(self, drone_ns):
        print("im in the call back")
        n_digits = 1
        def odom_callback(msg):
            position = msg.pose.pose.position
            self.positions[drone_ns] = (round(position.x,ndigits=n_digits) , round(position.y,ndigits=n_digits), round(position.z,ndigits=n_digits))
            print(f" {drone_ns} position {self.positions[drone_ns]}")
            self.adjust_followers(self.positions[drone_ns] , drone_ns)

        return odom_callback


    def move_drones_to_formation(self):

        twist_dic = dict()
        for i,content in enumerate(self.followers):
            twist_dic[content] = Twist()
            twist_dic[content].linear.y = 0.5
            self.velocity_publishers[content].publish(twist_dic[content])
        
        time.sleep(0.5)

        for i,content in enumerate(self.followers):
            twist_dic[content].linear.y = 0.0
            self.velocity_publishers[content].publish(twist_dic[content])

    def adjust_followers(self, position ,drone_ns):

        if (self.take_off):

            x_gap = 1
            y_gap = 2
            z_gap = 0

            gain = 0.5

            x = position[0]
            y = position[1]
            z = position[2]

            drone_xyz = dict()
            drone_twist = dict()

            for i,cont in enumerate(self.followers):

                if (cont == drone_ns):
                    follower_index = self.drone_namespaces.index(cont)
                    leader_index = self.drone_namespaces.index(self.leader_drone)
                    drone_twist[cont] = Twist()

                    if follower_index < leader_index:
                        index_i = self.drone_namespaces[0:leader_index].index(cont)
                        drone_xyz[cont+"_xyz"] = ((self.positions[self.leader_drone][0] - (x_gap*(index_i+1))),(self.positions[self.leader_drone][1] - (y_gap*(index_i+1))),(self.positions[self.leader_drone][2] - z_gap))
                    else:
                        index_i = self.drone_namespaces[leader_index+1:].index(cont)
                        drone_xyz[cont+"_xyz"] = ((self.positions[self.leader_drone][0] + (x_gap*(index_i+1))),(self.positions[self.leader_drone][1] - (y_gap*(index_i+1))),(self.positions[self.leader_drone][2] + z_gap))

                    if abs(self.positions[self.leader_drone][0]- x) > 10:
                        gain = 10
                    else:
                        gain = abs(self.positions[self.leader_drone][0]- x)

                    gain = gain/10

                    if x == drone_xyz[cont+"_xyz"][0]:
                        drone_twist[cont].linear.x = 0.0
                    elif (drone_xyz[cont+"_xyz"][0]) > x:
                        drone_twist[cont].linear.x = gain
                    elif (drone_xyz[cont+"_xyz"][0]) < x:
                        drone_twist[cont].linear.x = -gain
                    if drone_xyz[cont+"_xyz"][1] == y:
                        drone_twist[cont].linear.y = 0.0
                    elif (drone_xyz[cont+"_xyz"][1]) > y:
                        drone_twist[cont].linear.y = gain
                    elif (drone_xyz[cont+"_xyz"][1]) < y:
                        drone_twist[cont].linear.y = -gain
                    if drone_xyz[cont+"_xyz"][2] == z:
                        drone_twist[cont].linear.z = 0.0
                    elif (drone_xyz[cont+"_xyz"][2]) > z:
                        drone_twist[cont].linear.z = gain
                    elif (drone_xyz[cont+"_xyz"][2]) < z:
                        drone_twist[cont].linear.z = -gain

                    self.velocity_publishers[cont].publish(drone_twist[cont])
                    print(f"Publishing {cont}: {drone_twist[cont].linear.x,drone_twist[cont].linear.y,drone_twist[cont].linear.z}")
                    break

def launch_gazebo():
    subprocess.Popen(['ros2', 'launch', 'tello_gazebo', 'simple_launch.py'])

def main(args=None):

    # Initialize the ROS2 client library
    rclpy.init(args=args)

    # Instantiate DroneController with desired takeoff delay
    drone_controller = DroneController(takeoff_delay=0.1)  # Change the delay here
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
