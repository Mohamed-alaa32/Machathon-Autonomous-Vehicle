#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import time
import numpy as np
from std_msgs.msg import Float32

import asyncio
from bleak import BleakScanner, BleakClient
import keyboard  # Make sure to install this library first


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

class SolutionNode(Node):
    def __init__(self):
        super().__init__("subscriber_node")
        
        self.create_subscription(Float32, "/error", self.callback, 10)
      
        self.autonomous = True
        
        self.steeringPID = PID(0.8, 0.1, 0.8)

    def callback(self,error_msg:Float32):
        error = error_msg.data
        steering = self.steeringPID.step(error)  
        
        if self.autonomous:
            self.controlActions.publish(steering)
        
        




command_char_uuid = "00001142-0000-1000-8000-00805f9b34fb"

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
    
    while(True):
        asyncio.run(connect_and_control_rc_car(target_device_name)) 


def main():
    rclpy.init()
    node = SolutionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

