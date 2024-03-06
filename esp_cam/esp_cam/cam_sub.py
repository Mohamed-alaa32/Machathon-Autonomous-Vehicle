#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

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

class SolutionNode(Node):
    def __init__(self):
        super().__init__("ip_camera_node")
        ### Publisher to the image topic
        self.cam_publisher = self.create_publisher(Image,"/ip_camera",10)
        self.fps_counter = FPSCounter()
        self.bridge = CvBridge()

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


def main():
    rclpy.init()
    node = SolutionNode()
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