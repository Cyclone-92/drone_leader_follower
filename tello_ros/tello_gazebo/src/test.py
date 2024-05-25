import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from matplotlib.pyplot import plot as plt
from sensor_msgs.msg import Image
from tello_msgs.msg import TelloResponse
from cv_bridge import CvBridge
from tello_msgs.srv import TelloAction
from geometry_msgs.msg import Twist
import time
import signal
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import subprocess
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs
import math



class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        # initialize the QOS profile 
        self.policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # initialize the topics, actions and services required
        self.bridge = CvBridge()
        self.pose_publisher = self.create_publisher(Pose,'/gazebo/set_model_state', 10)
        self.odom = self.create_subscription(Odometry,"/drone1/odom", self.odom_callback, self.policy)
        self.response = self.create_subscription(TelloResponse, '/drone1/tello_response', self.respond, 10)
        self.response_code = 3
        self.client = self.create_client(TelloAction,"/drone1/tello_action")
        self.service_call_Takeoff()
        self.image_sub = self.create_subscription(Image, '/drone1/image_raw', self.image_callback, self.policy)
        self.publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        # initialize the desired flags, counters and variables
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
        """
                                        important!
        The below is the task alloacater which controlls the shapes and the colors
        in the Tuples 1st digit, 1 stands for square shape and 0 stands for circle shape
        2nd digit determines the color.each tuple is a one state
        """ 
        # shape == > circle : 0 square :1  # color == > red : 0    green  :1 orange : 2 
        self.control_seqence = [(1,0),(0,1),(1,2),(1,1),(1,1),(1,1),(1,1),(1,1),(1,1),(1,1),(1,1),(1,1),(1,1),(0,3)]
        # counter to check which state is in 
        self.seqence_tracker = 0
        # self.move("rotate_right")
        """
        end 
        """

# Odometry
#  Odemetry values
    def odom_callback(self,msg):
        print("im in the odom call back")
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.orientation = msg.pose.pose.orientation  # Quaternion orientation
        t3 = +2.0 * (self.orientation.w * self.orientation.z + self.orientation.x * self.orientation.y)
        t4 = +1.0 - 2.0 * (self.orientation.y * self.orientation.y + self.orientation.z * self.orientation.z)
        self.yaw  = math.atan2(t3, t4)

# Service related 
    def service_call_Takeoff(self):
        while not self.client.wait_for_service():
            self.get_logger().info("Waiting for the Service")
        else:
            self.get_logger().info(f"initiating take off")
            self.request = TelloAction.Request()
            self.response = TelloAction.Response()
            self.request.cmd = "takeoff"
            self.result = self.client.call_async(self.request)
            self.result.add_done_callback(self.callback_service_response)

    def service_call_land(self):
        self.get_logger().info(f"initiating landing")
        self.request = TelloAction.Request()
        self.request.cmd = "land"
        self.result = self.client.call_async(self.request)
        self.result.add_done_callback(self.callback_service_response)

    def callback_service_response(self, future):
        response = future.result()

    def respond(self,msg):
        self.response_code = msg.rc

# Main part for the image processing 
    def red_color_isolate(self,image):
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

        # fetching the shape and the color values from the control sequence
        if ((len(self.control_seqence)-1) >= self.seqence_tracker):
            shape,color  = self.control_seqence[self.seqence_tracker]
            print(f"sequence tracker {shape, color } state: {self.seqence_tracker} total state length {len(self.control_seqence)}")
        else:
        # if we have run out the states which means we are in the end
            print(" Service Ended - quiting the program")
        # if the shape 0 shape fine is set to 12, if one its set to 3
        if (shape == 0):
            shape_find = 12
        elif (shape == 1):
            shape_find = 4

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

            # I chekc if there are contours being a parent to a contour, if yes i take those
            if hierarchy[0][index][3] != -1:
                # Get the parent contour
                parent_contour_index = hierarchy[0][index][3]
                parent_contour = contours[parent_contour_index]     
                pt = tuple(contours[index][0][0].astype(float)) 
                # print(contour[0][0][0])
                result = cv2.pointPolygonTest(parent_contour, pt, False)
                # In t
                if result > 0:
                    if (0.5 <= h1/w1  and h1/w1 <= 2.0):
                        if(shape_find == 4):
                            # if ((len(approx) >= 0) or (len(approx) <= 8)):
                            # if ((len(approx) == 4)):
                            if ((area > 700) and (not self.wrong_direction)):
                                inside_contour.append(contour)
                                inside_area.append(area)
                                inside_shape.append(len(approx))
                        elif(shape_find == 12):
                            if ((len(approx) == 12)):
                                if ((area > 700) and (not self.wrong_direction)):
                                    inside_contour.append(contour)
                                    inside_area.append(area)
                                    inside_shape.append(len(approx))
                        
            #In herer we use sticker conditions to get exact one
            if (0.9 <= h1/w1  and h1/w1 <= 1.1):
                if ((len(approx) == shape_find)):
                    if ((area > 700) and (not self.wrong_direction) and (self.seqence_tracker < 3)):
                        areas.append(area)
                        contours_.append(contour)
                        shape_contour.append(len(approx))
                        # print(f"Detected contour h/w : {h/w} detected shape is : {len(approx)} detected area is : {area}")
            #WIn case of override this will be important
            if(color  == 3):
                if ((area > 300) and (not self.wrong_direction)):
                        areas.append(area)
                        contours_.append(contour)
                        shape_contour.append(len(approx))
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
            # if the found contour is a circle 
            if ((circle_found)):
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
                elif ((color  == 3) and green_go):
                    print(f"landing signal{self.landing_signal}")
                    # we move further close to land right on the pallete
                    if move_area <= 70000:
                        self.forward_count += 1
                        self.move("forward",1.0,0.0,1.0,0.01)
                    # if all got we call for the land service to land
                    else:
                        if (not self.landing_signal):
                            self.service_call_land()
                        while (self.response_code == 1):
                            print(f"Landing on the pallet")
                        else:
                            print(f"succesfully Landed")
                else:
                    # the object is more than 50000 and its not the stop sign, then its time to shoot the drone
                    error_threshhold = 5
                    # before shoot i wnna make sure im right on the centroid, by reducing the error margin
                    if ( ((errorx >= -error_threshhold) and (errorx <= error_threshhold)) and ((errory >= -error_threshhold) and (errory <= error_threshhold))):
                        # then I update a counter to make sure things are stable 
                        self.shoot_tracker += 1
                        if(self.shoot_tracker >=5):
                            window_area = width*height
                            # now if its a circle area is less so we have to adjust the shooting time
                            # P controller is used y = mx + x
                            # if the state is not the last one
                            if (self.seqence_tracker <=12):
                                if (shape == 0):
                                    shoot_time = (round(0.08/(move_area/window_area),2) + 0.05)
                                    shoot_time = max(min(shoot_time, 0.4), 0.25)
                                    print(f"\033[91mshoot completed :time , area , shape {shoot_time,move_area,filtered_shape}.\033[0m")
                                else:
                                    shoot_time = (round(0.08/(move_area/window_area),2))
                                    shoot_time = max(min(shoot_time, 0.4), 0.25)
                                    print(f"\033[91mshoot completed :time , area , shape {shoot_time,move_area,filtered_shape}.\033[0m")
                                # i wanna log out the data for fine tuning, so why not save it in a file
                                with open("output.txt", "+a") as file:
                                    file.write(f"{self.seqence_tracker} shoot completed :time , area , shape {shoot_time,move_area,filtered_shape}\n")
                                # before shoot make sure all the values are published to cms is set to 0
                                self.move("stop",1.0,1.0,0.1,0.01)
                                # shoot the drone
                                self.shoot(shoot_time)
                                self.go_back = False
                                self.shoot_tracker = 0
                            else:
                                # if the state is last one we land
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

# Image Call back
    def image_callback(self, msg):
        # Main image call back
        cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        # cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(cv_image,(640,480))
        # cv2.imwrite("original.jpg",image_resized)
        self.red_color_isolate(image_resized)


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
        
    def search_controller(self,time_wait):
        # simple 360 roation part
        timex_p = 1.0
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
            self.publisher_.publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher_.publish(twist)
        elif command == "back":
            # Go straight
            twist.linear.x = -timex_p
            twist.linear.y =  0.0
            twist.linear.z =  0.0
            self.publisher_.publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = -0.0
            self.publisher_.publish(twist)
        elif command == "left":
            # Go straight
            twist.linear.x = 0.0
            twist.linear.y = timex_p
            twist.linear.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher_.publish(twist)
        elif command == "right":
            # Go straight
            twist.linear.x =  0.0
            twist.linear.y = -timex_p
            twist.linear.z =  0.0
            self.publisher_.publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher_.publish(twist)
        elif command == "up":
            # Go straight
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = timey_p
            self.publisher_.publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher_.publish(twist)
        elif command == "down":
            # Go straight
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = -timey_p
            self.publisher_.publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher_.publish(twist)
        elif command == "stop_x":
            # Go straight
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
        elif command == "stop_y":
            # Go straight
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher_.publish(twist) 
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
            self.publisher_.publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = -0.0
            self.publisher_.publish(twist)
        elif command == "rotate_left":
            twist.linear.x  = 0.0
            twist.linear.y  = 0.0
            twist.linear.z  = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = timex_p
            self.publisher_.publish(twist)
            time.sleep(time_wait)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

    def shoot(self,time_wait):
        twist = Twist()
        # Go straight
        twist.linear.x = 1.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(time_wait)
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        self.publisher_.publish(twist)
        self.seqence_tracker += 1


def signal_handler(sig, frame):
    print('Ctrl+C detected. Ladning the drone...')
    ImageViewer().service_call_land()
    # Perform cleanup operations here if needed
    sys.exit(0)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    node = ImageViewer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()