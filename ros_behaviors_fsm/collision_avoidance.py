"""
Collision Avoidance (skeleton)
--------
Combines what used to be two separate behaviors:
- Reactive stopping (bump-triggered / very-close-range triggered), from the
  day 3 e-stop samples.
- Continuous steering around obstacles (potential fields) so the robot keeps
  moving and reroutes, rather than just halting, for anything that isn't an
  immediate emergency.

Design: bump or a critically-close reading (< stop_distance) is treated as a
hard-stop safety backstop -- no amount of steering is fast enough to matter
at that range, so just stop. Anything farther out but within
influence_radius contributes to a potential-fields steering command instead,
so the robot reroutes around obstacles it sees coming rather than plowing
forward and stopping only once it's already too close.

Fill in the TODOs to actually implement the potential-field force
computation and the force-to-steering conversion (see compute_potential_field
and process_scan below), and decide on any additional hard-stop conditions
called for by the assignment (e.g., a side-swipe trajectory).
"""
import math

import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker


class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Optional, but useful for seeing what the robot "thinks" in RViz:
        # Add -> By topic -> collision_avoidance_force -> Marker.
        self.marker_pub = self.create_publisher(Marker, 'collision_avoidance_force', 10)

        self.stop_distance = 0.3     # hard-stop trigger, in meters
        self.bumped = False
        self.too_close = False

        self.forward_speed = 0.1
        self.influence_radius = 1.0  # meters; obstacles farther than this are ignored
        self.k_attractive = 1.0      # TODO: tune
        self.k_repulsive = 1.0       # TODO: tune

    def process_bump(self, msg):
        self.bumped = bool(msg.left_front or msg.left_side or msg.right_front or msg.right_side)

    def process_scan(self, msg):
        front_range = msg.ranges[0]
        # TODO: consider more than a single range reading for robustness
        self.too_close = 0.0 < front_range < self.stop_distance

        if self.bumped or self.too_close:
            # Hard-stop safety backstop -- something is already too close
            # for steering around it to make sense.
            self.vel_pub.publish(Twist())
            return

        net_x, net_y = self.compute_potential_field(msg)

        # TODO: convert the net (net_x, net_y) force vector -- in the
        # robot's own frame, x = forward, y = left -- into an actual drive
        # command. Roughly: desired_heading = atan2(net_y, net_x); steer
        # proportionally toward it (angular.z), and drive forward at a
        # speed related to net force magnitude (e.g. slow down if net_x is
        # small/negative, meaning something is pushing back hard).
        vel = Twist()
        vel.linear.x = self.forward_speed
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)

        self.publish_force_marker(net_x, net_y)

    def compute_potential_field(self, msg):
        """Returns the net (x, y) force in the robot's frame: a constant
        forward attractive pull plus a repulsive contribution from every
        scan reading within self.influence_radius.

        TODO: implement the repulsive sum. For each valid range r at angle
        theta (0 = straight ahead) closer than self.influence_radius, add a
        force pointing from the obstacle back toward the robot -- i.e. in
        direction (-cos(theta), -sin(theta)) -- with magnitude that grows as
        r shrinks (e.g. k_repulsive * (1/r - 1/influence_radius)).
        """
        net_x, net_y = self.k_attractive, 0.0  # forward pull only, so far
        return net_x, net_y

    def publish_force_marker(self, x, y):
        """Visualizes the net force vector as an arrow from the robot's
        origin, in RViz.
        """
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # shaft diameter
        marker.scale.y = 0.1   # head diameter
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=x, y=y, z=0.0)]
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
