import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from matplotlib.pyplot import plot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tello_msgs.srv import TelloAction
from geometry_msgs.msg import Twist
import time
import signal
import sys

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.bridge = CvBridge()
        self.client = self.create_client(TelloAction,"/drone1/tello_action")
        self.service_call_Takeoff()
        # self.service_call_land()
        self.image_sub = self.create_subscription(Image, '/drone1/image_raw', self.image_callback, self.policy)
        self.publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        # self.move("rotate_right")


# Service related 
    def service_call_Takeoff(self):
        while not self.client.wait_for_service():
            self.get_logger().info("Waiting for the Service")
        else:
            self.get_logger().info(f"initiating take off")
            self.request = TelloAction.Request()
            self.request.cmd = "takeoff"
            self.result = self.client.call_async(self.request)
            self.result.add_done_callback(self.callback_service_response)

    def service_call_land(self):
        while not self.client.wait_for_service():
            self.get_logger().info("Waiting for the Service")
        else:
            self.get_logger().info(f"initiating landing")
            self.request = TelloAction.Request()
            self.request.cmd = "land"
            self.result = self.client.call_async(self.request)
            self.result.add_done_callback(self.callback_service_response)

    def callback_service_response(self, future):
        respose = future.result()
        if respose.rc == 3:
            self.get_logger().info(f"{self.request.cmd} not succesfull")
        elif respose.rc == 1:
            self.get_logger().info(f"{self.request.cmd} is Success!")

# Image related
    def red_color_isolate(self,image):
        print("Image processing started")
        hsv_image = cv2.cvtColor(image,cv2.COLOR_BGR2HLS)
        height, width, _ = hsv_image.shape
        # cv2.imwrite("hsv.jpg",hsv_image)
        # Define the lower and upper bounds for the red color range in HSV

        # orange
        # lower_red = np.array([10, error_margin, error_margin])
        # upper_red = np.array([40, 255, 255])


        # Green
        # lower_red = np.array([25, 52, 72])
        # upper_red = np.array([102, 255, 255])


        # Red
        lower_red = np.array([0, 0, 100])
        upper_red = np.array([10, 255, 255])


        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        # cv2.imwrite("masked.jpg",mask)
        # Dilate the green mask to fill in small holes
        kernel = np.ones((5, 5), np.uint8)
        green_mask_dilated = cv2.dilate(mask, kernel)
        # Find contours in the mask
        contours, _ = cv2.findContours(green_mask_dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Iterate through the contours
        cv2.namedWindow("image2")
        cv2.imshow('masked', green_mask_dilated)
        cv2.waitKey(1)
        nearest_area = []
        nearest_contour = []
        for contour in contours:
            # Get the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)
            # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            print(f"contour{h/w}")
            if (0.9 <= h/w  and h/w <= 1.2):
                area = cv2.contourArea(contour)
                nearest_area.append(area)
                nearest_contour.append(contour)
        #if deisred object found    
        if (len(nearest_area)> 0):
            # shape
            epsilon = 0.02 * cv2.arcLength(nearest_contour[np.argmax(nearest_area)], True)
            approx = cv2.approxPolyDP(nearest_contour[np.argmax(nearest_area)], epsilon, True)
            # Get the number of vertices in the approximation
            print(f"area {np.max(nearest_area)}")
            print(f"approx: {len(approx)}")
            if len(approx)==4:
                shape = "rectangle"
            elif len(approx)==8:
                shape = "circle"
            else:
                shape = "Cannot determine"
            print(f"shape { shape}")
            x, y, w, h = cv2.boundingRect(nearest_contour[np.argmax(nearest_area)])
            # Draw the bounding box on the original image
                # Calculate the moments of the contour
            M = cv2.moments(nearest_contour[np.argmax(nearest_area)])
            
            # Calculate the centroid coordinates
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            # Draw a circle at the centroid
            cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
            
            # Display the centroid coordinates
            cv2.putText(image, f'({cx}, {cy})', (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Calculate the centroid coordinates
            window_centroid_x = width // 2
            window_centroid_y = height // 2

            # Flight controller
            self.flight_contoller(window_centroid_x,window_centroid_y,cx,cy)

            # Plot a point (centroid) on the image
            cv2.circle(image, (window_centroid_x, window_centroid_y), 5, (255, 0, 0), -1)
            cv2.putText(image, f'({window_centroid_x}, {window_centroid_y})', (window_centroid_x + 10, window_centroid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.imshow('Red Blobs', image)
            cv2.waitKey(1)
        else:
            print(f"No Targeted object found, lets intiate search mode")

    def image_callback(self, msg):
        print("im in the call back")
        cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        # cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(cv_image,(640,480))
        # cv2.imwrite("original.jpg",image_resized)
        self.red_color_isolate(image_resized)

    # flight controller
    def flight_contoller(self,robotx1,roboty1,imagex1,imagey1):
        # lets calculate x  error
        error_x = robotx1 - imagex1
        error_y = roboty1 - imagey1
        print(f"error {error_x}")
        error_margin = 20
        if (error_x > -(error_margin)) and (error_x < error_margin):
            self.move("stop")
            print("Stop")
        elif error_x > error_margin:
            #turn left
            self.move("rotate_left")
            time.sleep(1)
            self.move("stop")
        elif error_x < -(error_margin):
            #turn right
            self.move("rotate_right")
            time.sleep(1)
            self.move("stop")
        else:
            self.move("stop")
            print("Stop")

        if (error_y > -(error_margin)) and (error_y < error_margin):
            self.move("stop")
            print("Stop")
        elif error_y > error_margin:
            #turn left
            self.move("up")
            time.sleep(1)
            self.move("stop")
        elif error_y < -error_margin:
            #turn right
            self.move("down")
            time.sleep(1)
            self.move("stop")
        else:
            self.move("stop")
            print("Stop")
         

# Controller Functionsm
    def move(self,command):
        twist = Twist()
        if command == "forward":
            # Go straight
            twist.linear.x = 0.01
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher_.publish(twist)
        elif command == "back":
            # Go straight
            twist.linear.x = -0.01
            twist.linear.y =  0.0
            twist.linear.z =  0.0
            self.publisher_.publish(twist)
        elif command == "left":
            # Go straight
            twist.linear.x = 0.0
            twist.linear.y = 0.1
            twist.linear.z = 0.0
            self.publisher_.publish(twist)
        elif command == "right":
            # Go straight
            twist.linear.x =  0.0
            twist.linear.y = -0.1
            twist.linear.z =  0.0
            self.publisher_.publish(twist)
        elif command == "up":
            # Go straight
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.09
            self.publisher_.publish(twist)
        elif command == "down":
            # Go straight
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = -0.09
            self.publisher_.publish(twist)
        elif command == "stop":
            # Go straight
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = -0.0
            self.publisher_.publish(twist)
        elif command == "rotate_right":
            # Go straight
            twist.linear.x  = 0.01
            twist.linear.y  = 0.0
            twist.linear.z  = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z = float(-0.5)
            self.publisher_.publish(twist)
        elif command == "rotate_left":
            # Go straight
            twist.linear.x  = 0.01
            twist.linear.y  = 0.0
            twist.linear.z  = 0.0
            twist.angular.x =  0.0
            twist.angular.y  = 0.0
            twist.angular.z =  0.5
            self.publisher_.publish(twist)
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