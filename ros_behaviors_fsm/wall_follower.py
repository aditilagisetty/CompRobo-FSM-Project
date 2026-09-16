"""
Wall Follower (skeleton)
--------
TODO: pilot the Neato to move forward while keeping its heading parallel to
the nearest wall, using proportional control on the angle/distance error
computed from two or more laser scan measurements.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_speed = 0.1

    def process_scan(self, msg):
        # TODO: pick two (or more) laser measurements to estimate the angle
        # between the robot's heading and the wall, e.g. msg.ranges[45] and
        # msg.ranges[135], and compute a proportional steering correction
        # from the error between them.
        vel = Twist()
        vel.linear.x = self.forward_speed
        vel.angular.z = 0.0  # TODO: replace with proportional control output
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
