#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import numpy as np

CAMERA_IP = "http://192.168.1.5" + ":81/stream"

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

class CamNode(Node):
    def __init__(self):
        super().__init__("ip_camera_node")
        ### Publisher to the image topic
        self.cam_publisher = self.create_publisher(Float32,"/error",10)
        self.fps_counter = FPSCounter()
        self.bridge = CvBridge()

        self.prevpt1 = [180, 80]
        self.prevpt2 = [660, 80]

    # Define a callback function to convert the image and publish it to a ROS topic
    def image_callback(self, img):
        try:
            # Convert the image to a ROS message
            ros_image = CvBridge().cv2_to_imgmsg(img, "bgr8")
            # Publish the image to a ROS topic
            self.cam_publisher.publish(ros_image)
            img = self.draw_fps(img)
            cv2.imshow("car_front",img)
        except CvBridgeError as e:
            print(e)

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
    
    def calcError(self, image):
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
                ptdistance[0] = abs(centroids[i][0] - self.prevpt1[0])
                ptdistance[1] = abs(centroids[i][0] - self.prevpt2[0])
                mindistance1.append(ptdistance[0])
                mindistance2.append(ptdistance[1])  

            threshdistance[0] = min(mindistance1)
            threshdistance[1] = min(mindistance2)
            
            minlb[0] = mindistance1.index(threshdistance[0])
            minlb[1] = mindistance2.index(threshdistance[1])

            cpt[0] = [centroids[minlb[0]+1][0], centroids[minlb[0]+1][1]]
            cpt[1] = [centroids[minlb[1]+1][0], centroids[minlb[1]+1][1]]

            if (threshdistance[0]>100):
                cpt[0] = self.prevpt1
            if (threshdistance[1]>100):
                cpt[1] = self.prevpt2

            mindistance1.clear()
            mindistance2.clear()

        else:
            # check if lane detected 
            cpt[0] = self.prevpt1
            cpt[1] = self.prevpt2

        self.prevpt1 = cpt[0]
        self.prevpt2 = cpt[1]

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
    return (error,self.prevpt1,self.prevpt2)



def main():
    rclpy.init()
    node = CamNode()
    # read frames from ip camera
    cap = cv2.VideoCapture(CAMERA_IP)
    while True:
        ret, frame = cap.read()
        if ret:
            node.image_callback(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    ##
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()