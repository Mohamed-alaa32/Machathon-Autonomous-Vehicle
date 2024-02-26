#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from prius_msgs.msg import Control
import time
import numpy as np

from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class FPSCounter:
    def __init__(self):
        self.frames = []

    def step(self):
        self.frames.append(time.monotonic())

    def get_fps(self):
        n_seconds = 5

        count = 0
        cur_time = time.monotonic()
        for f in self.frames:
            if cur_time - f < n_seconds:  # Count frames in the past n_seconds
                count += 1

        return count / n_seconds



# create node
def calcError(image , prevpt1, prevpt2):
    image_gray_ref = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # image_gray = image_gray_ref + 100 - image_gray_ref.mean()

    # ret, image_gray_thresh = cv2.threshold(image_gray, 0, 255, cv2.THRESH_BINARY)
    # detect red color
    lower_red = np.array([0, 0, 100])
    upper_red = np.array([100, 100, 255])
    mask = cv2.inRange(image, lower_red, upper_red)
    image_gray_thresh = cv2.bitwise_and(image, image, mask=mask)
    image_gray_thresh = cv2.cvtColor(image_gray_thresh, cv2.COLOR_BGR2GRAY)
    #use thresholding to get binary image
    ret, image_gray_thresh = cv2.threshold(image_gray_thresh, 0, 255, cv2.THRESH_BINARY)
    # remove noise
    kernel = np.ones((5,5),np.uint8)
    image_gray_thresh = cv2.erode(image_gray_thresh,kernel,iterations = 1)
    image_gray_thresh = cv2.dilate(image_gray_thresh,kernel,iterations = 1)


    cv2.imshow("thresh", image_gray_thresh)
    
    #Crop lower third of the image
    height, width = image_gray_thresh.shape
    dst = image_gray_thresh[int(2*height/3):height, 0:width]

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(dst.astype(np.uint8), connectivity=4, ltype=cv2.CV_32S)
    ptdistance = np.zeros(3)

    mindistance1 = []
    mindistance2 = []
    threshdistance = [0,0]
    minlb = [0,0]
    cpt = [[0,0],[0,0]]

    if num_labels > 1:
        for i in range(1, num_labels):
            ptdistance[0] = abs(centroids[i][0] - prevpt1[0])
            ptdistance[1] = abs(centroids[i][0] - prevpt2[0])
            mindistance1.append(ptdistance[0])
            mindistance2.append(ptdistance[1])  

        threshdistance[0] = min(mindistance1)
        threshdistance[1] = min(mindistance2)
        
        minlb[0] = mindistance1.index(threshdistance[0])
        minlb[1] = mindistance2.index(threshdistance[1])

        cpt[0] = [centroids[minlb[0]+1][0], centroids[minlb[0]+1][1]]
        cpt[1] = [centroids[minlb[1]+1][0], centroids[minlb[1]+1][1]]

        if (threshdistance[0]>100):
            cpt[0] = prevpt1
        if (threshdistance[1]>100):
            cpt[1] = prevpt2

        mindistance1.clear()
        mindistance2.clear()

    else:
        # check if lane detected 
        cpt[0] = prevpt1
        cpt[1] = prevpt2

    prevpt1 = cpt[0]
    prevpt2 = cpt[1]

    fpt = [(cpt[0][0] + cpt[1][0])/2, (cpt[0][1] + cpt[1][1])/2 + int(2*height/3)]

    # Visualize fpt
    cv2.circle(image, (int(fpt[0]), int(fpt[1])), 5, (0, 0, 255), -1)
    cv2.circle(image, (int(cpt[0][0]), int(cpt[0][1])+int(2*height/3)), 5, (0, 255, 0), -1)
    cv2.circle(image, (int(cpt[1][0]), int(cpt[1][1])+int(2*height/3)), 5, (0, 255, 0), -1)


    cv2.circle(dst, (int(fpt[0]), int(fpt[1])), 5, (255, 255, 255), -1)
    dst = cv2.cvtColor(dst.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    cv2.imshow("dst", dst)


    error = width/2 - fpt[0]
    error = (error*90.0/400)/15
    return (error,prevpt1,prevpt2)

class SolutionNode(Node):
    def __init__(self):
        super().__init__("subscriber_node")
        ### Subscriber to the image topic
        self.subscriber = self.create_subscription(Image,"/prius/front_camera/image_raw",self.callback,10)
        self.subscriber_odom = self.create_subscription(Odometry, "/prius/odom", self.odom_callback, 10)
        ### Publisher to the control topic
        self.publisher = self.create_publisher(Control, "/prius/control", qos_profile=10)
        self.fps_counter = FPSCounter()
        
        self.bridge = CvBridge()
        self.command = Control()
        self.state = Odometry()
        self.autonomous = True

        self.prevpt1 = [180, 80]
        self.prevpt2 = [660, 80]

    def odom_callback(self,msg:Odometry):
        self.state = msg
    def throttleControl(self, error:float) ->float:
        """
        If delta Z is positive or angle of elevation is high, throttle more
        and if steering angle is sharp, throttle less
        and if abs(error) is high, throttle less

        if want to slow down, apply brake (negative throttle) and make condition if throttleAction is negative, put data in brake msg
        if want to speed up, apply throttle
        """
        #See which angle is the angle of elevation
        # roll , pitch , yaw = euler_from_quaternion([self.state.pose.pose.orientation.x, self.state.pose.pose.orientation.y, self.state.pose.pose.orientation.z, self.state.pose.pose.orientation.w])
        
        vx = self.state.twist.twist.linear.x
        vy = self.state.twist.twist.linear.y
        vz = self.state.twist.twist.linear.z
        kp = 0.5
        ki = 0.5
        currentSpeed = math.sqrt(vx**2 + vy**2 + vz**2)
        targetSpeed = (5/(0.0001+abs(error))) #+ kp * pitch
        targetSpeed = min(10, targetSpeed)
        throttleAction = (targetSpeed - currentSpeed) * ki                       #pitch * k1 - abs(steering) * k2 + abs(error) * k3
        # self.get_logger().info(f"Target Speed: {targetSpeed}")
        throttleAction = max(-1.0, min(1.0, throttleAction))
        #Throttle is limited to 0 to 1 (1 is maximum throttle)
        #Brake is limited to 0 to 1 (1 is maximum brake)
        #<max_speed>37.998337013956565</max_speed>
            #<max_steer>0.6458</max_steer>
        return throttleAction
    
    def draw_fps(self, img):
        self.fps_counter.step()
        fps = self.fps_counter.get_fps()
        cv2.putText(
            img,
            f"FPS: {fps:.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        return img

    def callback(self,msg:Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = self.draw_fps(cv_image)
        error, self.prevpt1, self.prevpt2 = calcError(cv_image, self.prevpt1, self.prevpt2)
        # print(error)
        # self.get_logger().info(f"Error: {error} prevpt1: {self.prevpt1} prevpt2: {self.prevpt2}")  
        
        ctrl_msg = Control()
        ctrl_msg.steer = np.clip(error, -1, 1)
        # self.get_logger().info(f"Steering: {ctrl_msg.steer}")
        throttle = self.throttleControl(error)
        if throttle < 0:
            ctrl_msg.brake = abs(throttle) * 0.5
        else:
            ctrl_msg.throttle = throttle
        ctrl_msg.shift_gears = 2
        # ctrl_msg.throttle = #0.15
        if self.autonomous:
            self.publisher.publish(ctrl_msg)

        #### show image
        cv2.imshow("prius_front",cv_image)
        cv2.waitKey(5)

def main():
    rclpy.init()
    node = SolutionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()