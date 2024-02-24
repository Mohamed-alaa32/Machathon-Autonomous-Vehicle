import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped
import sys

class Control(Node):
    def __init__(self):
        super().__init__('waypoints_node', namespace='waypoints')
        self.get_logger().info('Waypoints Node Started')

        self.create_subscription(Odometry, '/prius/odom', self.odom_callback, 10)

        self.waypoints = Path()
        self.waypointsPub = self.create_publisher(Path, '/waypoints', 10)
        self.index = 0
    def odom_callback(self, msg:Odometry):
        
        #When you have collected 500 waypoints, publish the waypoints
        if self.index == 500:
            self.waypointsPub.publish(self.waypoints)
        
        
        else:
            point = PoseStamped()
            point.pose.position.x =  msg.pose.pose.position.x
            point.pose.position.y =  msg.pose.pose.position.y
            self.index += 1
            self.waypoints.poses.append(point)
        
            

        
        

def main(args=None):
    rclpy.init(args=args)
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)