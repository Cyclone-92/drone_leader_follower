"""
# Aerial Robotics UTU 2024: Our Common Project
 
## Description:
This code is meant to take off a trello drone, then move trough 
green check points using images from a on-board camera in a 
gazebo simulation, using ROS2 and OpenCV.

## Installation:
Based on: https://github.com/TIERS/drone_racing_ros2/tree/main
Run this on your console once: (or VisualStudio Code terminal, if you rather)

    mkdir -p ~/drone_racing_ros2_ws/src
    cd ~/drone_racing_ros2_ws/src
    git clone https://github.com/TIERS/drone_racing_ros2.git
    cd ..
    source /opt/ros/galactic/setup.bash
    colcon build

## Usage
Run this every time you want to run the gazeboenvironment

    cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    ros2 launch tello_gazebo simple_launch.py

Then, run "OpenProjectAerialRobotics.py" with python from console

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

## Control the Drone

    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1    

Enjoy!
 
## Authors: 
     - Julian C. Paez P. [julian.c.paezpineros@utu.fi]
     - Michalis Iona [michalis.l.iona@utu.fi]
     - Prashan Herath [prashan.r.herathmudiyanselage@utu.fi]
 
For: University of Turku - TIERS 
Course: Aerial Robotics and Multi-Robot Systems 
Date: March 26th, 2024 

## Changes
Works only for a single green square checkpoint
note: PENDING TO CALIBRATE CIRCLE BY num_vertices

"""

import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from tello_msgs.srv import TelloAction
import time
from geometry_msgs.msg import Twist
import numpy as np

class OpenProjectAerialRobotics:
    def __init__(self):
        print("Initializing node...")
        self.node = rclpy.create_node('OpenProjectAerialRobotics')
        self.bridge = CvBridge()

        # Create subscriber for receiving image data from the drone camera
        print("Subscribing to camera service...")
        self.subscription = self.node.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data)
        
        # Create client for sending takeoff and land commands to the drone
        self.cli = self.node.create_client(TelloAction, '/drone1/tello_action')

        # Create publisher for sending Twist messages
        self.publisher_ = self.node.create_publisher(Twist, '/drone1/cmd_vel', 10)

        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available yet, waiting...')  
        
        # Create request message for takeoff command
        self.req = TelloAction.Request()
        self.req.cmd = 'takeoff'  # Set the command to 'takeoff'
        self.future = self.cli.call_async(self.req)
        print("Attempting to take off...")
        time.sleep(4)  # give a little time to little parrot drone to take off



    def image_callback(self, msg):

        # Convert ROS image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Resize the image
        new_width = 450
        new_height = 300
        image = cv2.resize(image, (new_width, new_height))

        # Convert color from BGR to RGB for better visualization
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Apply Gaussian blur
        #hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

        # Define range of green, red and orange colors in HSV
        lower_all = np.array([0, 185, 0]) #([0, 69, 0])
        upper_all = np.array([179, 255, 100])

        # Define range of green color in HSV
        # lower_green = np.array([40, 40, 40])
        # upper_green = np.array([80, 255, 255])

        # # Define range of red color in HSV
        # lower_red = np.array([0, 40, 40])
        # upper_red = np.array([40, 255, 255])

        # # Define range of orange color in HSV
        # lower_orange = np.array([11, 100, 100])
        # upper_orange = np.array([25, 255, 255])

        # Threshold the HSV image to get only selected colors
        mask = cv2.inRange(hsv, lower_all, upper_all)
        #red_mask = cv2.inRange(hsv, lower_green, upper_green)
        #red_mask = cv2.inRange(hsv, lower_red, upper_red)
        #orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Combine masks for green, red, and orange
        #mask = cv2.bitwise_and(cv2.bitwise_and(green_mask, red_mask), orange_mask)
        #mask = cv2.addWeighted(orange_mask, 0.5, green_mask, 0.5, 0)

        # # Load image, grayscale, median blur, sharpen image, Threshold and morph close
        #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(mask, 11)
        sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpen = cv2.filter2D(blur, -1, sharpen_kernel)
        #thresh = cv2.threshold(sharpen, 160, 255, cv2.THRESH_BINARY_INV)[1]
        #kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        #close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Erode image
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        eroded  = cv2.erode(sharpen, kernel, iterations=3)

        # Dilate image (Desuse)
        #dilated = cv2.dilate(eroded, kernel, iterations=2)

        # # Perform Canny edge detection (Desuse)
        # edges = cv2.Canny(mask, 50, 150)

        # Find contours of green objects
        contours, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Create a blank image with the same dimensions as the original image
        contour_image = np.zeros_like(image)

        # Draw contours on the blank image
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 1)

        # Convert contour image to 8-bit depth
        #contour = cv2.convertScaleAbs(contour)
        
        # Image dimensions
        height, width = image.shape[:2]

        # Initialize variables to store center of the largest contour
        largest_x = None
        largest_y = None
        max_area = -1
        
        # Initialize movement variables
        allowed_centering_error = 10 # To avoid hovering too much arround the center
        cmd_publish = Twist()        

        # Analize area of every contour
        for contour in contours:

            # Get area of contour
            area = cv2.contourArea(contour)

            if area > 1: #500 and area < 6000:

                # Calculate the perimeter of the contour
                perimeter = cv2.arcLength(contour, True)
                
                # Approximate the contour to a polygon
                epsilon = 0.05 * perimeter
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Get number of vertices of such polygon
                num_vertices = len(approx)

                # Continue only if polygon has 4 vertices "Square" or a lot of them "Circle" 
                # Note: PENDING TO CALIBRATE CIRCLE BY num_vertices
                if True: #num_vertices > 3 or num_vertices > 100:

                    (x, y, w, h) = cv2.boundingRect(approx)
                    aspect_ratio = w / float(h)
                    
                    #print(aspect_ratio) ################################  HELP! ##############################################

                    # Set a threshold for the aspect ratio to filter out rectangles
                    if True: #aspect_ratio >= 0.7 and aspect_ratio <= 1.3:

                        # Keep track of largest shape (area-wise) and obtain radious
                        if area > max_area:
                            max_area = area
                            (largest_x, largest_y), radius = cv2.minEnclosingCircle(contour)
                            largest_contour = contour

                        # Fit a circle to each shape and its center
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        # centerE, axes, angle = cv2.fitEllipse(contour)
                        center = (int(x), int(y))
                        radius = int(radius)
                        cv2.circle(imageRGB, center, radius, (0, 255, 0), 2)  # Circle in centroid
                        cv2.circle(imageRGB, center, 10, (0, 255, 0), -1)  # Circle in center  

        # Follow largest shape if it exists

        # if  cmd_publish.linear.x == 0.0 and cmd_publish.linear.y == 0.0 and cmd_publish.linear.z == 0.0 and cmd_publish.angular.x == 0.0 and cmd_publish.angular.y == 0.0 and cmd_publish.angular.z == 0.0:
            
        #     print("Not moving. Readjusting...")
        #     cmd_publish.linear.x = -0.1
        #     cmd_publish.linear.y = 0.0
        #     cmd_publish.linear.z = 0.0
        #     cmd_publish.angular.x = 0.0
        #     cmd_publish.angular.y = 0.0
        #     cmd_publish.angular.z = 0.0

        if largest_x == None or largest_y == None:

            print("No shapes found. Turning right...")
            cmd_publish.linear.x = -0.005
            cmd_publish.linear.y = 0.0
            cmd_publish.linear.z = 0.0
            cmd_publish.angular.x = 0.0
            cmd_publish.angular.y = 0.0
            cmd_publish.angular.z = -1.0
            self.publisher_.publish(cmd_publish)
   
        else:
            
            # State current area of shape
            #print("area of ",largest_contour,"is = ", max_area)

            # Compute the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Calculate the center of the rectangle
            center_x = x + w / 2
            center_y = y + h / 2

            margin = 20

            # Check if the center is close to any of the borders within 20 pixels
            if (center_x <= margin or center_x >= width - margin or center_y <= margin or center_y >= height - margin):
                print("Center of the contour is within 20 pixels of a border.")
            else:
                # Draw cyan vertices in polygon
                for vertex in largest_contour:
                    x, y = vertex[0]
                    cv2.circle(imageRGB, (x, y), 5, (0, 255, 255), -1)
            
                # Draw a purple circle in the center of the biggest contour
                cv2.circle(imageRGB, (int(largest_x), int(largest_y)), 10, (255, 0, 255), -1)

                # Follow biggest contour horizontally
                center_x_image = image.shape[1] // 2
                center_y_image = image.shape[0] // 2
                if largest_x > center_x_image + allowed_centering_error:
                    print("Turning right...",max_area)
                    cmd_publish.linear.x = -0.002
                    cmd_publish.linear.y = 0.0
                    cmd_publish.linear.z = 0.0
                    cmd_publish.angular.x = 0.0
                    cmd_publish.angular.y = 0.0
                    cmd_publish.angular.z = -0.02 # Turn right
                elif largest_x < center_x_image - allowed_centering_error:
                    print("Turning left...",max_area)
                    cmd_publish.linear.x = 0.0
                    cmd_publish.linear.y = 0.0
                    cmd_publish.linear.z = 0.0
                    cmd_publish.angular.x = 0.0
                    cmd_publish.angular.y = 0.0
                    cmd_publish.angular.z = 0.02  # Turn left
                else:
                    cmd_publish.angular.z = 0.0  # Stop steering

                # Follow biggest contour vertically
                    if largest_y < center_y_image - allowed_centering_error:
                        print("Moving up...", max_area)
                        cmd_publish.linear.x = 0.0
                        cmd_publish.linear.y = 0.0
                        cmd_publish.linear.z = 0.02  # Move up
                        cmd_publish.angular.x = 0.0
                        cmd_publish.angular.y = 0.0
                        cmd_publish.angular.z = 0.0
                    elif largest_y > center_y_image + allowed_centering_error:
                        print("Moving down...", max_area)
                        cmd_publish.linear.x = 0.0
                        cmd_publish.linear.y = 0.0
                        cmd_publish.linear.z = -0.02  # Move down
                        cmd_publish.angular.x = 0.0
                        cmd_publish.angular.y = 0.0
                        cmd_publish.angular.z = 0.0
                    else:
                        # Here a condition for follow GTA checkpoint blindly when countour area is greater than some value should be aded
                        if max_area > 11000 and max_area < 15000:
                            print("Accelerating to pass GTA Checkpoint blindly... WISH ME LUCK :O",max_area)
                            cmd_publish.linear.x = 2.0 # Move forward
                            cmd_publish.linear.y = 0.0
                            cmd_publish.linear.z = 0.0
                            cmd_publish.angular.x = 0.0
                            cmd_publish.angular.y = 0.0
                            cmd_publish.angular.z = 0.0
                            # time.sleep(1)
                            # cmd_publish.linear.x = 0 # Move forward
                        else:
                            print("Drone is centered. Moving forward... :)",max_area)
                            cmd_publish.linear.x = 0.15  # Move forward
                            cmd_publish.linear.y = 0.0
                            cmd_publish.linear.z = 0.0   # Stop leveling
                            cmd_publish.angular.x = 0.0
                            cmd_publish.angular.y = 0.0
                            cmd_publish.angular.z = 0.0

                        

        # Publish movement commands
        self.publisher_.publish(cmd_publish)

        # Display the images
        # cv2.imshow('image', image)
        # cv2.imshow('mask', mask)
        # cv2.imshow('hsv', hsv)
        # cv2.imshow('eroded', eroded)
        # cv2.imshow('dilated', dilated)
        # cv2.imshow('contour', contour)

        # cv2.moveWindow('image', 0, 0)
        # cv2.moveWindow('mask', 525, 0)
        # cv2.moveWindow('hsv', 980, 0)
        # cv2.moveWindow('eroded', 0, 420)
        # cv2.moveWindow('dilated', 525, 420) 
        # cv2.moveWindow('contour', 980, 420)

        cv2.imshow('imageRGB', imageRGB)
        cv2.imshow('mask', mask)
        cv2.imshow('blur', blur)
        cv2.imshow('sharpen', sharpen)
        cv2.imshow('eroded', eroded)
        cv2.imshow('contour_image', contour_image)

        cv2.moveWindow('imageRGB', 0, 0)
        cv2.moveWindow('mask', 525, 0)
        cv2.moveWindow('blur', 980, 0)
        cv2.moveWindow('sharpen', 0, 420)
        cv2.moveWindow('eroded', 525, 420) 
        cv2.moveWindow('contour_image', 980, 420)
        
        cv2.waitKey(1)

        # Add a little pause for me to see what is happening
        time.sleep(0.01)

    # def stop_drone(self, area, threshold=0.1):
    #     # Calculate the error between the area of the gate and the threshold
    #     error = np.abs(0.4 - area) 
    #     print(f"Error: {error}")
    #     # Calculate the steps to move
    #     steps = error * self.speedx
    #     # Check if the drone is close enough to the gate
    #     if area < threshold:
    #         self.move_x(steps)
    #         self.moving = True
    #         self.close_enough = False
    #         self.centered = False
    #     else:
    #         self.close_enough = True
    #         self.moving = False
    #         self.stop()
    #         if self.sim:
    #             self.send_request_simulator('land')
    #         else:
    #             self.land()
    #         exit()
    #     return


def main(args=None):
    rclpy.init(args=args)
    Open_Project_Aerial_Robotics = OpenProjectAerialRobotics()
    rclpy.spin(Open_Project_Aerial_Robotics.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()