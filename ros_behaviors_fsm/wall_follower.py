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
        self.distance_from_wall = 1.0
        self.kp = (
            0.5  # TODO: tune this proportional gain to get good wall-following behavior
        )

    def process_scan(self, msg):
        # TODO: pick two (or more) laser measurements to estimate the angle
        # between the robot's heading and the wall, e.g. msg.ranges[45] and
        # msg.ranges[135], and compute a proportional steering correction
        # from the error between them.
        vel = Twist()

        front_dist = msg.ranges[0]

        if (
            not math.isnan(front_dist)
            and not math.isinf(front_dist)
            and front_dist > 0.0
        ):
            # if the front distance is less than equal to 1.0 meters, stop the robot and turn left
            if front_dist <= 1.0:
                vel.linear.x = 0.0
                vel.angular.z = -1.0
                self.vel_pub.publish(vel)
                return

        if any(math.isinf(r) or math.isnan(r) for r in msg.ranges[45:136]):
            vel.linear.x = self.forward_speed
            vel.angular.z = 0.1
            self.vel_pub.publish(vel)
            return

        error1 = msg.ranges[90] - self.distance_from_wall
        error_allign = msg.ranges[45] - msg.ranges[135]

        # Calculation for the error in allignment along the wall
        vel.linear.x = float(self.forward_speed)
        vel.angular.z = float((self.kp * error1) + (self.kp * error_allign))

        # DEbug print statements
        print(f"Angular Z: {vel.angular.z}")

        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
