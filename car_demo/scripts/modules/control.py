import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry,Path
from prius_msgs.msg import Control
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32  
from tf_transformations import euler_from_quaternion
import math

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node', namespace='controller')
        self.get_logger().info('Controller Node Started')

        self.create_subscription(Odometry, '/prius/odom', self.odom_callback, 10)

        self.waypoints = Path()
        self.waypointsPub = self.create_subscription(Path, '/waypoints', 10)

        self.create_subscription(Float32, '/error', self.error_callback, 10)
        # self.controlPub = self.create_publisher(Control, '/prius/control', 10)

        self.state = Odometry()
        self.error = 0.0
        self.prevError = 0.0

    def odom_callback(self, msg:Odometry):
        self.state = msg

    def error_callback(self, msg:Float32):
        self.error = msg.data

    def searchTargetIndex(self):

        for i in range(len(self.waypoints.poses)):
            dx = self.waypoints.poses[i].pose.position.x - self.pose.pose.position.x
            dy = self.waypoints.poses[i].pose.position.y - self.pose.pose.position.y
            d = math.sqrt(dx**2 + dy**2)
            if d > 1:
                return i
    
    def steerPidControl(self, error:float) ->float:
        """
        if error is positive, steer right
        if error is negative, steer left
        
        """
        self.totalError += error
        steerAction = error * kp #+ (error - self.prevError) * kd + (self.totalError) * ki
        
        self.prevError = error

        steerAction = max(-1.0, min(1.0, steerAction))
        #Steering rate is limited to -1 to 1 (+1 is maximum left turn, -1 is maximum right turn)
        # Max steering angle is 0.6458 radian (37 degrees)
        return steerAction
    
    def throttleControl(self, state:Odometry, steering:float, error:float) ->float:
        """
        If delta Z is positive or angle of elevation is high, throttle more
        and if steering angle is sharp, throttle less
        and if abs(error) is high, throttle less

        if want to slow down, apply brake (negative throttle) and make condition if throttleAction is negative, put data in brake msg
        if want to speed up, apply throttle
        """
        #See which angle is the angle of elevation
        roll , pitch , yaw = euler_from_quaternion([state.pose.pose.orientation.x, state.pose.pose.orientation.y, state.pose.pose.orientation.z, state.pose.pose.orientation.w])
        
        vx = state.twist.twist.linear.x
        vy = state.twist.twist.linear.y
        kp = 0.5
        ki = 0.5
        currentSpeed = math.sqrt(vx**2 + vy**2)
        targetSpeed = (1/(0.0001+abs(steering))) + kp * pitch
        
        throttleAction = (targetSpeed - currentSpeed) * ki                       #pitch * k1 - abs(steering) * k2 + abs(error) * k3

        throttleAction = max(-1.0, min(1.0, throttleAction))
        #Throttle is limited to 0 to 1 (1 is maximum throttle)
        #Brake is limited to 0 to 1 (1 is maximum brake)
        #<max_speed>37.998337013956565</max_speed>
            #<max_steer>0.6458</max_steer>
        return throttleAction
    
    def controlMsg(self) ->Control:
        control = Control()
        steering = self.steerPidControl(self.error)
        throttle = self.throttleControl(self.state, steering, self.error)
        control.steer = steering
        
        if throttle < 0:
            control.brake = abs(throttle) * 0.5
        else:
            control.throttle = throttle
        control.shift_gears = 2
        
        return control

    def safetyCheck(self, state):
        """
        If the car is still in it's place for a long time, reverse the car
        """

    def controlActions(self) ->Control:
        targetIndex = self.searchTargetIndex()
        control = Control()
        control.shift_gears = 2
        control.throttle = 0.3
        control.steer = 0.3
        return control
"""
std_msgs/Header header

# Range 0 to 1, 1 is max throttle
float64 throttle
# Range 0 to 1, 1 is max brake
float64 brake
# Range -1 to +1, +1 is maximum left turn
float64 steer

uint8 NO_COMMAND=0
uint8 NEUTRAL=1
uint8 FORWARD=2
uint8 REVERSE=3

uint8 shift_gears

"""
    
