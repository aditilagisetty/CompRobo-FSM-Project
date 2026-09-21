import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class FSMNode(Node):
    def __init__(self):
        super().__init__("fsm_node")

        self.state = "WALL FOLLOW"  # Initial state

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscription for different nodes to send velocity commands
        self.create_subscription(Twist, "cmd_vel_wall_follower", self.wall_follower, 10)
        self.create_subscription(
            Twist, "cmd_vel_obstacle_avoidance", self.obstacle_avoidance, 10
        )

        # Subscrption for LaserScan data
        self.create_subscription(LaserScan, "scan", self.run_loop, 10)

    def wall_follower(self, msg):
        if self.state == "WALL FOLLOW":
            self.vel_pub.publish(msg)

    def obstacle_avoidance(self, msg):
        if self.state == "OBSTACLE AVOIDANCE":
            self.vel_pub.publish(msg)

    def run_loop(self, msg):
        # Check for obstacles in front of the robot
        front_cone = msg.ranges[:10] + msg.ranges[-10:]
        valid_ranges = [r for r in front_cone if math.isfinite(r) and r > 0.0]
        front_distance = min(valid_ranges) if valid_ranges else float("inf")

        # print the current state for debugging purpose
        print(self.state)

        if (
            front_distance < 0.5 and self.state == "WALL FOLLOW"
        ):  # If an obstacle is detected within 0.5 meters
            self.state = "OBSTACLE AVOIDANCE"
        elif (
            front_distance > 0.7 and self.state == "OBSTACLE AVOIDANCE"
        ):  # leeway to prevent rapid switching
            self.state = "WALL FOLLOW"


def main(args=None):
    rclpy.init(args=args)
    fsm_node = FSMNode()
    rclpy.spin(fsm_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
