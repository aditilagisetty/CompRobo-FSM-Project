"""
Follow the closer wall, steering away from anything straight ahead.

Publish an RViz marker at the point on the wall the robot is using to
steer.
"""

import math

from geometry_msgs.msg import Point, Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class WallFollower(Node):
    """
    Follow a wall using laser scan data.

    The robot follows whichever wall (left or right) is closer, and
    turns away from anything detected straight ahead to avoid a collision.
    """

    def __init__(self):
        """Initialize speed, gains, and state used while following a wall."""
        super().__init__('wall_follower')
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_wall_follower', 10)
        # as an arrow in RViz, the same way collision_avoidance.py
        # visualizes its net force on collision_avoidance_force.
        self.marker_pub = self.create_publisher(Marker, 'wall_detection_marker', 10)
        self.forward_speed = 0.1
        self.distance_from_wall = 1.0
        self.kp = 0.5
        self.follow_side = (
            1  # +1 for left wall, -1 for right wall, None for no wall detected
        )

        self.is_turning = False
        self.turn_start_time = None
        self.turn_speed = 0.4  # rad/s
        self.turn_duration = (math.pi / 2.0) / self.turn_speed

        self.has_teleop_run = False  # Flag to check if teleop has run before

    def process_scan(self, msg):
        """Pick a wall to follow from the scan and publish the next velocity."""
        # Initial values declared
        vel = Twist()
        now = self.get_clock().now().nanoseconds / 1e9
        front_dist = msg.ranges[0]

        if math.isfinite(front_dist) and 0.0 < front_dist <= 1.0:
            self.is_turning = True
            self.turn_start_time = now
            vel.linear.x = 0.0
            vel.angular.z = float(-1.0 * self.follow_side * self.turn_speed)
            self.vel_pub.publish(vel)
            return

        r45 = msg.ranges[45]
        r90 = msg.ranges[90]
        r135 = msg.ranges[135]
        r225 = msg.ranges[225]
        r270 = msg.ranges[270]
        r315 = msg.ranges[315]

        decided_angles = []

        r90_valid = math.isfinite(r90) and r90 > 0.0
        r270_valid = math.isfinite(r270) and r270 > 0.0

        # Follow whichever wall is closer; if both are equally close, follow
        # the left wall.
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

        if any(
            math.isinf(r) or math.isnan(r)
            for r in [decided_angles[0], decided_angles[1], decided_angles[2]]
        ):
            vel.linear.x = self.forward_speed
            vel.angular.z = float(self.follow_side * 0.1)
            self.vel_pub.publish(vel)
            return

        error1 = decided_angles[1] - self.distance_from_wall

        # Calculation for the error in allignment along the wall
        if self.follow_side == 1:
            # Left Wall
            error_align = decided_angles[0] - decided_angles[2]
            steering = (self.kp * error1) + (self.kp * error_align)
        else:
            # Right Wall
            error_align = decided_angles[2] - decided_angles[0]
            steering = -1.0 * ((self.kp * error1) + (self.kp * error_align))

        vel.linear.x = float(self.forward_speed)
        vel.angular.z = float(steering)

        # DEbug print statements
        print(f"Side: {'LEFT' if self.follow_side == 1 else 'RIGHT'}")
        print(f'Angular Z: {vel.angular.z}')

        self.publish_wall_marker(decided_angles[1], 90 * self.follow_side)
        self.vel_pub.publish(vel)

    def publish_wall_marker(self, distance, angle_deg):
        """Visualizes the detected wall point as an arrow from the robot's origin."""
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # shaft diameter
        marker.scale.y = 0.1  # head diameter
        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=x, y=y, z=0.0)]
        self.marker_pub.publish(marker)


def main(args=None):
    """Initialize rclpy, spin the WallFollower node, then shut down."""
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
