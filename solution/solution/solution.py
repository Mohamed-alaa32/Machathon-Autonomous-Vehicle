#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import time
import numpy as np
from std_msgs.msg import Float32

import asyncio
from bleak import BleakScanner, BleakClient
import keyboard  # Make sure to install this library first
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import numpy as np

CAMERA_IP = "http://192.168.1.5" + ":81/stream"
command_char_uuid = "00001142-0000-1000-8000-00805f9b34fb"

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

class CamClass():
    def __init__(self):
        ### Publisher to the image topic
        self.fps_counter = FPSCounter()
        self.prevpt1 = [180, 80]
        self.prevpt2 = [660, 80]
        self.error = 0
        self.steeringAngle = 0
        self.maxSteer = 30

    def getSteeringAngle(self):
        self.steeringAngle = np.clip(self.error,-1,1) * self.maxSteer

    # Define a callback function to convert the image and publish it to a ROS topic
    def image_callback(self, img):
        try:            
            self.error = self.calcErrorFromColor(img)
            # self.error = self.calcErrorFromEdge(img)

            # Publish the image to a ROS topic
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
    
    def calcErrorFromColor(self, image):
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
        return (error)
   
    def calcErrorFromEdge(self,image_in):
        gray = cv2.cvtColor(image_in, cv2.COLOR_BGR2GRAY)

        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray,(kernel_size,kernel_size),0)
        cv2.imshow("blur",blur_gray)
        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

        scharrx = cv2.Scharr(edges, cv2.CV_16S, 1, 0)
        scharry = cv2.Scharr(edges, cv2.CV_16S, 0, 1)
        scharrx = cv2.convertScaleAbs(scharrx)
        scharry = cv2.convertScaleAbs(scharry)
        scharr = cv2.bitwise_or(scharrx,scharry)

        dilated = cv2.dilate(scharr, (9,9), iterations=5)
        
        image_gray_thresh = dilated.copy()

        image_gray_thresh[60:,:] = 0
        #Crop lower third of the image
        height, width = image_gray_thresh.shape
        # dst = image_gray_thresh[int(2*height/3)-30:height, 0:width]
        dst = image_gray_thresh[0:height, 0:width]
        ### LEFT LANE
        dstLeft = image_gray_thresh[0:height, 0:width//3]

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(dstLeft.astype(np.uint8), connectivity=4, ltype=cv2.CV_16U)
        ptdistance = np.zeros(3)

        mindistance1 = []
        threshdistance = [0,0]
        minlb = [0,0]
        cpt = [[0,0],[0,0]]

        if num_labels > 1:
            for i in range(1, num_labels):
                ptdistance[0] = abs(centroids[i][0] - self.prevpt1[0])
                mindistance1.append(ptdistance[0])

            threshdistance[0] = min(mindistance1)
            
            minlb[0] = mindistance1.index(threshdistance[0])

            cpt[0] = [centroids[minlb[0]+1][0], centroids[minlb[0]+1][1]]

            if (threshdistance[0]>30):
                cpt[0] = self.prevpt1

            mindistance1.clear()
        else:
            # check if lane detected 
            cpt[0] = self.prevpt1

        

        dstRight = image_gray_thresh[0:height, 0:width]
        dstRight[:,0:2*width//3] = 0
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(dstRight.astype(np.uint8), connectivity=4, ltype=cv2.CV_16U)
        ptdistance = np.zeros(3)

        if num_labels > 1:
            for i in range(1, num_labels):
                ptdistance[1] = abs(centroids[i][0] - self.prevpt2[0])
                mindistance1.append(ptdistance[1])

            threshdistance[1] = min(mindistance1)
            
            minlb[1] = mindistance1.index(threshdistance[1])

            cpt[1] = [centroids[minlb[1]+1][0], centroids[minlb[1]+1][1]]

            if (threshdistance[1]>30):
                cpt[1] = self.prevpt2

            mindistance1.clear()
        else:
            # check if lane detected 
            cpt[1] = self.prevpt2
        
            

        self.prevpt1 = cpt[0]
        self.prevpt2 = cpt[1] 

        fpt = [(cpt[0][0] + cpt[1][0])/2, (cpt[0][1] + cpt[1][1])/2 ]

        # Visualize fpt
        # cv2.circle(image, (int(fpt[0]), int(fpt[1])), 5, (0, 0, 255), -1)
        # cv2.circle(image, (int(cpt[0][0]), int(cpt[0][1])), 5, (255, 0, 0), -1)
        # cv2.circle(image, (int(cpt[1][0]), int(cpt[1][1])), 5, (0, 255, 0), -1)
        # cv2.circle(dst, (int(fpt[0]), int(fpt[1])), 5, (255, 255, 255), -1)

        dst = cv2.cvtColor(dst.astype(np.uint8), cv2.COLOR_GRAY2BGR)


        error = width/2 - fpt[0]
        error = (error*90.0/400)/15
        return error

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.derivative = 0
        self.prev_time = time.monotonic()
        self.time = self.prev_time

    def step(self, error):
        self.time = time.monotonic()
        dt = self.time - self.prev_time
        self.integral += error * dt
        self.derivative = (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_time = self.time
        #needs to be changed
        return np.clip(self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative, -1, 1)

async def run_ble_client(device_address: str):
    # await send_variables(device_address, speed, steeringAngle)
    async with BleakClient(device_address) as client:
        if client.is_connected:
            print(f"Connected to {device_address}")
            print("Control the car: \nW/S: Accelerate/Decelerate\nA/D: Turn Left/Right\nESC: Quit")
            # while True:
            #     #await asyncio.sleep(0.1)  # Prevents the loop from using too much CPU
            # # Send the byte array over BLE
            #     if keyboard.is_pressed('esc'):  # Quit if ESC is pressed
            #         print("Quitting...")
            #         break
            #     if keyboard.is_pressed('w'):
            #         data = b'W' + speed.to_bytes(1,byteorder='little') + steeringAngle.to_bytes(1, byteorder='little')
            #         await client.write_gatt_char(command_char_uuid, data)
            #         print("Forward")
            #          # Wait for key release
            #         await asyncio.sleep(0.1)
            #     elif keyboard.is_pressed('s'):
            #         data = b'S' + speed.to_bytes(1, byteorder='little') + steeringAngle.to_bytes(1, byteorder='little')
            #         await client.write_gatt_char(command_char_uuid,data)
            #         print("Backward")
            #         await asyncio.sleep(0.1)
            #     if keyboard.is_pressed('a'):
            #         data = b'A' + speed.to_bytes(1, byteorder='little') + steeringAngle.to_bytes(1, byteorder='little')
            #         await client.write_gatt_char(command_char_uuid,data)
            #         print("Left")
            #         #await asyncio.sleep(0.0001)
            #     elif keyboard.is_pressed('d'):
            #         data = b'D' + speed.to_bytes(1, byteorder='little') + steeringAngle.to_bytes(1, byteorder='little')
            #         await client.write_gatt_char(command_char_uuid,data)
            #         print("Right")
                    #await asyncio.sleep(0.0001)swwwwwwwwwwwwwws
                #else:
                #s     await client.write_gatt_char(command_char_uuid, b'S')

async def discover_device(target_device_name: str):
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name and target_device_name.lower() in device.name.lower():
            print(f"Found target device: {device.name}, Address: {device.address}")
            return device.address
    return None

async def connect_and_control_rc_car(target_device_name: str):
    device_address = await discover_device(target_device_name)
    if device_address is not None:
        await run_ble_client(device_address)
    else:
        print("Target device not found.")

async def send_variables(device_address: str, speed: int, steeringAngle: int):
    async with BleakClient(device_address) as client:
        if client.is_connected:
            # Package the variables into a byte array
            data = bytes([speed, steeringAngle])
            # Send the byte array over BLE
            await client.write_gatt_char("00001142-0000-1000-8000-00805f9b34fb", data)
        
if __name__ == "__main__":
    target_device_name = "testing"  # Name of the BLE device (Arduino)
    speed = 10
    steeringAngle = 10    
    camera = CamClass()
    # read frames from ip camera
    cap = cv2.VideoCapture(CAMERA_IP)
    
    while(True):
        asyncio.run(connect_and_control_rc_car(target_device_name)) 
        ret, frame = cap.read()
        if ret:
            camera.image_callback(frame)
        steeringAngle = camera.getSteering()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
