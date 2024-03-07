import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped,Pose
import sys

from prius_msgs.msg import Control

from tf_transformations import euler_from_quaternion

class control(Node):
    def __init__(self):
        super().__init__('waypoints_node', namespace='waypoints')
        self.get_logger().info('Waypoints Node Started')

        self.create_subscription(Odometry, '/prius/odom', self.odom_callback, 10)

        self.waypoints = Path()
        self.waypointsPub = self.create_publisher(Path, '/waypoints', 10)
        self.anglePub = self.create_publisher(Pose, '/angle', 10)

        self.controlPub = self.create_publisher(Control, '/prius/control', 10)

        self.index = 0
    def odom_callback(self, msg:Odometry):
        
        #When you have collected 500 waypoints, publish the waypoints
        # if self.index == 500:
        #     self.waypointsPub.publish(self.waypoints)
        pitch, roll, yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        
        angle = Pose()
        angle.position.x = pitch
        angle.position.y = roll
        angle.position.z = yaw
        self.anglePub.publish(angle)
        
        control = Control()

        control.throttle = 1.0
        control.shift_gears = Control.FORWARD
        self.controlPub.publish(control)
        # else:
        #     point = PoseStamped()
        #     point.pose.position.x =  msg.pose.pose.position.x
        #     point.pose.position.y =  msg.pose.pose.position.y
        #     self.index += 1
        #     self.waypoints.poses.append(point)
        
            

        
        

def main(args=None):
    rclpy.init(args=args)
    node = control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)