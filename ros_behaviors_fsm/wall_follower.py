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
        self.follow_side = (
            None  # +1 for left wall, -1 for right wall, None for no wall detected
        )

    def process_scan(self, msg):
        # TODO: pick two (or more) laser measurements to estimate the angle
        # between the robot's heading and the wall, e.g. msg.ranges[45] and
        # msg.ranges[135], and compute a proportional steering correction
        # from the error between them.
        vel = Twist()

        front_dist = msg.ranges[0]

        r45 = msg.ranges[45]
        r90 = msg.ranges[90]
        r135 = msg.ranges[135]
        r225 = msg.ranges[225]
        r270 = msg.ranges[270]
        r315 = msg.ranges[315]

        decided_angles = []

        # This checks what wall is close to the neato and sets the follow side accordingly. If both walls are close, it will follow the left wall.
        r90_valid = math.isfinite(r90) and r90 > 0.0
        r270_valid = math.isfinite(r270) and r270 > 0.0

        # This checks what wall is close to the neato and sets the follow side accordingly. If both walls are close, it will follow the left wall.
        if r90_valid and r270_valid:
            if r90 <= r270:
                self.follow_side = 1  # Left side is closer
            else:
                self.follow_side = -1  # Right side is closer
        elif r90_valid:
            self.follow_side = 1  # Only left wall visible
        elif r270_valid:
            self.follow_side = -1  # Only right wall visible

        # Assign the angles to be used for wall following based on the follow side
        if self.follow_side == 1:
            decided_angles = [r45, r90, r135]
        else:
            decided_angles = [r225, r270, r315]

        if (
            not math.isnan(front_dist)
            and not math.isinf(front_dist)
            and front_dist > 0.0
        ):
            # If the front distance is less than or equal to 1.0 meters, stop and turn away from the wall
            if front_dist <= 1.0:
                vel.linear.x = 0.0
                vel.angular.z = float(
                    self.follow_side * -0.1
                )  # Turn left or right based on follow side
                self.vel_pub.publish(vel)
                return

        if any(
            math.isinf(r) or math.isnan(r)
            for r in [decided_angles[0], decided_angles[1], decided_angles[2]]
        ):
            vel.linear.x = self.forward_speed
            vel.angular.z = float(self.follow_side * 0.1)
            self.vel_pub.publish(vel)
            return

        error1 = decided_angles[1] - self.distance_from_wall
        error_allign = decided_angles[0] - decided_angles[2]

        # Calculation for the error in allignment along the wall
        vel.linear.x = float(self.forward_speed)
        vel.angular.z = float(
            self.follow_side * ((self.kp * error1) + (self.kp * error_allign))
        )

        # DEbug print statements
        print(f"Side: {'LEFT' if self.follow_side == 1 else 'RIGHT'}")
        print(f"Angular Z: {vel.angular.z}")

        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
