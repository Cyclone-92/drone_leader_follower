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
import sys
import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import subprocess
import re
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self._previous_error = 0
        self._integral = 0

    def update(self, current_value):
        error = self.setpoint - current_value
        self._integral += error
        derivative = error - self._previous_error

        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        self._previous_error = error

        return output

class DroneController(Node):
    def __init__(self, takeoff_delay=2):
        super().__init__('drone_controller')
        self.takeoff_delay = takeoff_delay  # Time in seconds between each drone's takeoff
        self.policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        print("Starting the Simulation")
        self.bridge = CvBridge()
        result = subprocess.run(['ros2','topic','list'], capture_output=True, text=True)

        drone_values = re.findall(r'drone\d+', result.stdout)

        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        self.orientation = 0
        self.odom_z_counter = 0
        self.z_flag = 0
        self.z_value = 0
        self.x_flag = 0
        self.x_value = 0
        self.y_flag = 0
        self.y_value = 0
        self.go_back = False
        self.forward_count = 0
        self.shoot_tracker = 0
        self.seqence_tracker = 0
        self.wrong_direction = False
        self.object_count = 0
        self.global_count = 0
        self.search_roate_counter = 0
        self.no_global_onj = [1]
        self.kernal = 5
        self.landing_signal = False

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

        self.image_sub = self.create_subscription(Image, f"/{self.leader_drone}/image_raw", self.image_callback, self.policy)
        self.positions = {ns: None for ns in self.drone_namespaces}
        self.last_drone2_pos = None
        
        # Take off each drone sequentially with a delay
        self.takeoff_sequentially()


    def leader(self,arr):
        n = len(arr)
        if n == 0:
            sys.exit(-1)
        if n % 2 == 1:
            middle_index = n // 2
            return arr[middle_index]
        else:
            middle_index1 = n // 2 - 1
            print(middle_index1)
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

    def service_call_land(self):
        for ns in self.drone_namespaces:
            req = TelloAction.Request()
            req.cmd = 'land'
            self.get_logger().info(f'Attempting to land {ns}...')
            future = self.action_clients[ns].call_async(req)
            # # Wait for the future to complete


    def odom_callback_factory(self, drone_ns):
        print("im in the call back")
        n_digits = 1
        def odom_callback(msg):
            position = msg.pose.pose.position
            self.positions[drone_ns] = (round(position.x,ndigits=n_digits) , round(position.y,ndigits=n_digits), round(position.z,ndigits=n_digits))
            print(f" {drone_ns} position {self.positions[drone_ns]}")
            self.adjust_followers(self.positions[drone_ns] , drone_ns)
        return odom_callback

    def image_callback(self, msg):
        if self.take_off:
            # Main image call back
            cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
            # cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
            image_resized = cv2.resize(cv_image,(640,480))
            # cv2.imwrite("original.jpg",image_resized)
            self.image_processor(image_resized)
        else:
            print("image processor is waiting for drones to fully take off")


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

    def adjust_followers(self, position, drone_ns):
        """
        Adjusts the position of follower drones relative to the leader drone.

        Args:
            position (tuple): The current position (x, y, z) of the drone.
            drone_ns (str): The namespace identifier for the drone.
        """

        # Check if the drones have taken off
        if self.take_off:
            
            # Set the gaps between the drones
            x_gap = 1
            y_gap = 2
            z_gap = 0

            # Unpack the position coordinates
            x, y, z = position

            # Initialize dictionaries to store positions and velocities of drones
            drone_xyz = dict()
            drone_twist = dict()

            # PID controllers for each axis
            pid_x = PID(Kp=0.5, Ki=0.0, Kd=0.1)
            pid_y = PID(Kp=0.5, Ki=0.0, Kd=0.1)
            pid_z = PID(Kp=0.5, Ki=0.0, Kd=0.1)

            for i, cont in enumerate(self.followers):

                if cont == drone_ns:
                    follower_index = self.drone_namespaces.index(cont)
                    leader_index = self.drone_namespaces.index(self.leader_drone)
                    drone_twist[cont] = Twist()

                    if follower_index < leader_index:
                        index_i = self.drone_namespaces[0:leader_index].index(cont)
                        drone_xyz[cont+"_xyz"] = (
                            self.positions[self.leader_drone][0] - (x_gap * (index_i + 1)),
                            self.positions[self.leader_drone][1] - (y_gap * (index_i + 1)),
                            self.positions[self.leader_drone][2] - z_gap
                        )
                    else:
                        index_i = self.drone_namespaces[leader_index + 1:].index(cont)
                        drone_xyz[cont+"_xyz"] = (
                            self.positions[self.leader_drone][0] + (x_gap * (index_i + 1)),
                            self.positions[self.leader_drone][1] - (y_gap * (index_i + 1)),
                            self.positions[self.leader_drone][2] + z_gap
                        )

                    # Set the setpoints for the PID controllers
                    pid_x.setpoint = drone_xyz[cont+"_xyz"][0]
                    pid_y.setpoint = drone_xyz[cont+"_xyz"][1]
                    pid_z.setpoint = drone_xyz[cont+"_xyz"][2]

                    # Compute the control signals using the PID controllers
                    drone_twist[cont].linear.x = pid_x.update(x)
                    drone_twist[cont].linear.y = pid_y.update(y)
                    drone_twist[cont].linear.z = pid_z.update(z)

                    # Publish the calculated velocity to the current drone
                    self.velocity_publishers[cont].publish(drone_twist[cont])
                    print(f"Publishing {cont}: {drone_twist[cont].linear.x, drone_twist[cont].linear.y, drone_twist[cont].linear.z}")

                    break

    def leader_path (self,target_pos, self_pos):
        pass

    def image_processor(self,image):
        # print("Image processing started")
        hsv_image = cv2.cvtColor(image,cv2.COLOR_BGR2HLS)
        detected_contour = image.copy()
        filterd_contour  = image.copy()
        height, width, _ = hsv_image.shape
        window_centroid_x = width // 2
        window_centroid_y = height // 2
        areas = []
        contours_ = []
        shape_contour = []
        inside_contour = []
        inside_area = []
        inside_shape = []
        found_object = False
        no_obj_area = []
        circle_found  = False
        green_go = False

        lower_stop = np.array([50, 0, 64])
        upper_stop = np.array([255, 53, 255])
    
        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv_image, lower_stop, upper_stop)

        # Dilate the green mask to fill in small holes
        fill_kerenl = np.ones((10, 10), np.uint8)
        kernel = np.ones((3, 3), np.uint8)
        # Do morphological closing 
        filled_image = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, fill_kerenl)
        eroded_image = cv2.erode(filled_image, kernel, iterations=1)
        lurred_image = cv2.GaussianBlur(eroded_image, (3, 3), 0)

        # Find contours in the mask
        contours, hierarchy = cv2.findContours(lurred_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Morpho operators
        for index,contour in enumerate(contours):
            # Get the bounding box of the contour
            x1, y1, w1, h1 = cv2.boundingRect(contour)
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            area = cv2.contourArea(contour)

            # Calculate the moments of the contour
            M_main = cv2.moments(contour[0])
            # Calculate the centroid coordinates
            if M_main["m00"] != 0:
                cx_main = int(M["m10"] / M["m00"])
                cy_main = int(M["m01"] / M["m00"])
            else:
                cx_main, cy_main = 0, 0
            no_obj_area.append(area)

            if ((area > 10)):
                inside_contour.append(contour)
                inside_area.append(area)
                inside_shape.append(len(approx))

                        
            #In herer we use sticker conditions to get exact one
            if (0.9 <= h1/w1  and h1/w1 <= 1.1):
                if ((area > 700)):
                    areas.append(area)
                    contours_.append(contour)
                    shape_contour.append(len(approx))
                    # print(f"Detected contour h/w : {h/w} detected shape is : {len(approx)} detected area is : {area}")
            # Draw all the contours 
            cv2.rectangle(filterd_contour, (x1, y1), (x1 + w1, y1 + h1), (0, 0, 255), 2)
            cv2.imshow('object found', filterd_contour)
            cv2.waitKey(1)
            cv2.resizeWindow('Original Image', 800, 600) 
        # If we found any filtered contours we perform this 
        if (len(contours_)> 0):
            # Bigest area contour 
            index = np.argsort(areas)[::-1]
            # print(f"\033[92mfiltered 01 contours found : {len(contours_)}, shape : {shape_contour[index[0]]} area : {areas[index[0]]}\033[0m")
            filtered_shape = shape_contour[index[0]]
            x, y, w, h = cv2.boundingRect(contours_[index[0]])
            # Calculate the moments of the contour
            M = cv2.moments(contours_[0])
            # Calculate the centroid coordinates
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            found_object = True
            move_area = areas[index[0]]
            self.object_count = 0
            self.global_count = 0
            self.no_global_onj.append(1)
        # if we find something in hirearchy method we use this
        elif (len(inside_contour) > 0):
            # Bigest area contour 
            index = np.argsort(inside_area)[::-1]
            # print(f"\033[92mfiltered contours 02 found : {len(inside_contour)} area : {inside_area} shape : {inside_shape} Rank index : {index}\033[0m")
            filtered_shape = inside_shape[index[0]]
            x, y, w, h = cv2.boundingRect(inside_contour[index[0]])
            # Calculate the moments of the contour
            M = cv2.moments(inside_contour[index[0]])
            # Calculate the centroid coordinates
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            found_object = True
            move_area = inside_area[index[0]]
            self.object_count = 0
            self.global_count = 0
            self.no_global_onj.append(1)
        else:
            # If nothing founds we set the flag
            found_object = False
        # if we found an object
        if(found_object):
            # print(f"total len og glob is {len(self.no_global_onj)} and count is {self.forward_count} kernel is {self.kernal}")
            # This is an experimateal code to make adaptive thresholding. 
            if (len(self.no_global_onj) == 100):
                # print(f"forward count is {self.forward_count} detected move objects {np.sum(self.no_global_onj)}kernel is {self.kernal}")
                if ((np.sum(self.no_global_onj) == 100) and (self.forward_count < 10)):
                    self.kernal = 7
                    self.no_global_onj = [1]
                    self.forward_count = 0
                else:
                    self.kernal = 5
                    self.no_global_onj = [1]
            elif ((self.forward_count > 10)):
                self.kernal = 5
                self.forward_count = 0
                self.no_global_onj = [1]
            # if object found
            cv2.circle(filterd_contour, (window_centroid_x, window_centroid_y), 5, (255, 0, 0), -1)
            cv2.putText(filterd_contour, f'({window_centroid_x}, {window_centroid_y})', (window_centroid_x + 10, window_centroid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.circle(filterd_contour, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(filterd_contour, f'({cx}, {cy})', (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.rectangle(filterd_contour, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.imshow('object found', filterd_contour)
            cv2.waitKey(1)

            # This is the drone shooting part and navigation part

            # now we check the error. calculates the centroid of the object found and centroid of the video frame, 
            # then we calculate how deviated we are, this is the error
            errorx, errory = self.flight_contoller(window_centroid_x,window_centroid_y,cx,cy,move_area)
            # error thresh hold is set to 20, 
            error_threshhold = 20
            if ( ((errorx >= -error_threshhold) and (errorx <= error_threshhold)) and ((errory >= -error_threshhold) and (errory <= error_threshhold))):
                # if the area is less than 50000 we move closer
                if move_area <= 50000:
                    self.forward_count += 1
                    self.move("forward",1.0,0.0,1.0,0.01)
                # we are pretty close, now if the detected object is stop sign
                else :
                    print(f"landing signal{self.landing_signal}")
                    # we move further close to land right on the pallete
                    if move_area <= 70000:
                        self.forward_count += 1
                        self.move("forward",1.0,0.0,1.0,0.01)
                    # if all got we call for the land service to land
                    else:
                        if (not self.landing_signal):
                            self.service_call_land()
        else:
            # Here comes means the computer vison could find a right contour
            #  Search algorithm
            self.object_count += 1
            if (self.object_count == 0):
                self.go_back = False 
                self.z_flag = 0
                self.x_flag = 0
            # we turn a 360 turn
            print(self.z_flag ,self.go_back)
            if (not self.go_back):
                # Rotate
                if (self.z_flag == 0):
                    self.z_value = round((round(self.yaw,1) - 0.1),1)
                    self.z_flag = 1
                if (self.z_flag == 1):
                    if (self.z_value != round(self.yaw,1)):
                        print("search rotation initiated")
                        print(f"z_value {self.z_value} and crrent degree {round(self.yaw,1)}")
                        self.search_controller(0.02)
                        self.go_back = False 
                    else:
                        print("Rotation Stoped!")
                        self.move("stop",0.0,0.0,0.0,0.1)
                        self.go_back = True 
                        self.z_flag = 0
            #if the 360 turn is done and still no obbject may be its time to take a step back
            elif((self.go_back )and (not self.landing_signal)):
                # Going back by one grid 
                if (self.x_flag == 0):
                    # self.x_value = round(self.x - grid_x_size * math.cos(self.yaw), 1)
                    # self.y_value = round(self.y - grid_x_size * math.sin(self.yaw), 1)
                    self.x_flag = 1
                if (self.x_flag == 1):
                    print("search back initiated")
                    self.move("back",1.0,0.0,0.0,0.2)
                    self.go_back = False
                    self.x_flag = 0
            cv2.imshow('object found', filterd_contour)
            cv2.waitKey(1)

    def search_controller(self,time_wait):
        # simple 360 roation part
        timex_p = 0.5
        timey_p = 0.0
        timef_p = 0.0
        self.move("rotate_left",timex_p,timey_p,timef_p,time_wait)


# Controller Functionsm
    def move(self,command,timex_p,timey_p,timef_p,time_wait):
        #  This is where values are publish to cmd topic
        twist = Twist()
        print(command,timex_p,timey_p,timef_p,time_wait)
        if command == "forward":
            # Go straight
            twist.linear.x = timef_p
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
        elif command == "back":
            # Go straight
            twist.linear.x = -timex_p
            twist.linear.y =  0.0
            twist.linear.z =  0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = -0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
        elif command == "left":
            # Go straight
            twist.linear.x = 0.0
            twist.linear.y = timex_p
            twist.linear.z = 0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
        elif command == "right":
            # Go straight
            twist.linear.x =  0.0
            twist.linear.y = -timex_p
            twist.linear.z =  0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
        elif command == "up":
            # Go straight
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = timey_p
            self.velocity_publishers[self.leader_drone].publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
        elif command == "down":
            # Go straight
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = -timey_p
            self.velocity_publishers[self.leader_drone].publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
        elif command == "stop_x":
            # Go straight
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
        elif command == "stop_y":
            # Go straight
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.velocity_publishers[self.leader_drone].publish(twist) 
        elif command == "stop":
            twist.linear.x  = 0.0
            twist.linear.y  = 0.0
            twist.linear.z  = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = 0.0
        elif command == "rotate_right":
            # Go straight
            twist.linear.x  = 0.0
            twist.linear.y  = 0.0
            twist.linear.z  = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = -timex_p
            self.velocity_publishers[self.leader_drone].publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = -0.0
            self.velocity_publishers[self.leader_drone].publish(twist)
        elif command == "rotate_left":
            twist.linear.x  = 0.0
            twist.linear.y  = 0.0
            twist.linear.z  = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = timex_p
            self.velocity_publishers[self.leader_drone].publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = 0.0
            self.velocity_publishers[self.leader_drone].publish(twist)

    def flight_contoller(self,robotx1,roboty1,imagex1,imagey1,move_area):
        #  This is flight controler algorith
        # lets calculate x  error
        error_x = robotx1 - imagex1
        error_y = roboty1 - imagey1
        # max x = 320
        # max y = 240
        # lets calculate the speed
        if(move_area > 10000):
            timex_p = round(float(abs(error_x/320)*1.0),2)+0.05
            timey_p = round(float(abs(error_y/240)*1.0),2)+0.05
        else:
            timex_p = round(float(abs(error_x/320)*1),2)+0.01
            timey_p = round(float(abs(error_y/240)*1),2)+0.01
        # if the values are too big, we can clip off
        if timex_p > 1.0:
            timex_p = 1.0
        if timey_p > 1.0:
            timey_p = 1.0
        timef_p = 0.0
        time_wait = 0.02

        error_margin = 5
    #   Navigate the drone 
        if (error_x > -(error_margin)) and (error_x < error_margin) :
            print(f"error x , error y  {error_x,error_y} and speed x, y, time {timex_p, timey_p,time_wait} :stop_x")
            self.move("stop_x",timex_p,timey_p,timef_p,time_wait)
            # print("stop_x")
        elif error_x > error_margin:
            #turn left
            time_wait = 0.01
            print(f"error x , error y  {error_x,error_y} and speed x, y, time {timex_p, timey_p,time_wait} :rotate_left")
            self.move("rotate_left",timex_p,timey_p,timef_p,time_wait)
        elif error_x < (error_margin):
            #turn right
            time_wait = 0.01
            print(f"error x , error y  {error_x,error_y} and speed x, y, time {timex_p, timey_p,time_wait} :rotate_Right")
            self.move("rotate_right",timex_p,timey_p,timef_p,time_wait)

        if (error_y > -(error_margin)) and (error_y < error_margin):
            # stop y
            print(f"error x , error y  {error_x,error_y} and speed x, y, time {timex_p, timey_p,time_wait} :stop_y")
            self.move("stop_y",timex_p,timey_p,timef_p,time_wait)
        elif error_y > error_margin:
            # print("up ")
            print(f"error x , error y  {error_x,error_y} and speed x, y, time {timex_p, timey_p,time_wait} :up")
            self.move("up",timex_p,timey_p,timef_p,time_wait)
        elif error_y < error_margin:
            # print("down")
            print(f"error x , error y  {error_x,error_y} and speed x, y, time {timex_p, timey_p,time_wait} :down")
            self.move("down",timex_p,timey_p,timef_p,time_wait)


        return error_x,error_y


def launch_gazebo():
    subprocess.Popen(['ros2', 'launch', 'tello_gazebo', 'openproject.py'])

def main(args=None):

    # launch_gazebo()

    # time.sleep(6)
    # Initialize the ROS2 client library
    rclpy.init(args=args)

    # Instantiate DroneController with desired takeoff delay
    drone_controller = DroneController(takeoff_delay=0.1)  # Change the delay here
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
