import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.forward_speed = 0.1
        self.distance_from_wall = 0.3
        self.kp = (
            0.5  # TODO: tune this proportional gain to get good wall-following behavior
        )

    def process_scan(self, msg):
        # TODO: pick two (or more) laser measurements to estimate the angle
        # between the robot's heading and the wall, e.g. msg.ranges[45] and
        # msg.ranges[135], and compute a proportional steering correction
        # from the error between them.
        vel = Twist()

        if (
            msg.ranges[270] == self.distance_from_wall
            and msg.ranges[315] == self.distance_from_wall / math.cos(math.pi / 2)
            and msg.ranges[225] == self.distance_from_wall / math.cos(math.pi / 2)
        ):
            vel.linear.x = self.forward_speed
        else:
            # TODO: replace with proportional control output
            error1 = msg.ranges[270] - self.distance_from_wall
            error_allign = msg.ranges[315] - msg.ranges[225]

            # Calculation for the error in allignment along the wall
            vel.linear.x = self.forward_speed
            vel.angular.z = -(self.kp * error1) - (self.kp * error_allign)
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
