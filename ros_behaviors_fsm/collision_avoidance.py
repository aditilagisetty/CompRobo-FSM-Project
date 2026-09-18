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

    # convert a desired angle to an actual reading we can use because 
    # LaserScan stores msg.list as a flat list so reading i 
    # corresponds to the angle msg.angle_min + i * msg.angle_increment
    # so we want a function that does the reverse --> given an angle
    # find the index in ranges that corresponds to it
    def _range_at_angle(self, msg, degrees):
        """Looks up the scan range closest to the degrees from the 
        robots forward direction where 0 is straight ahead, 90 is 
        facing left, and -90 is right
        
        Returns range, returns inf for missing readings."""

        angle_rad = math.radians(degrees)
        # back calculate to find range index
        index = int(round((angle_rad - msg.angle_min) / msg.angle_increment))
        # forces index to always be in valid range
        index %= len(msg.ranges)
        r = msg.ranges[index]
        # return of 0 means there is no obsticle detected rather than
        # there is an obstacle at distance 0 so return inf
        return r if r > 0.0 else float('inf')

    # do the _range_at_angle function for a large spread of angles
    def _min_range_in_cone(self, msg, center_deg, half_width_deg):
        """Smallest valid range within plus or minus half_width_deg of
        center_deg such that a dropped reading cant hide an obstacle.
        """
        readings = [
            self._range_at_angle(msg, center_deg + offset)
            for offset in range(-half_width_deg, half_width_deg + 1)
        ]
        return min(readings)

    def process_bump(self, msg):
        self.bumped = bool(msg.left_front or msg.left_side or msg.right_front or msg.right_side)

    def process_scan(self, msg):
        self.front_range = self._min_range_in_cone(msg, center_deg=0, half_width_deg=10)
        self.too_close = self.front_range < self.stop_distance


        if self.bumped or self.too_close:
            # Hard-stop safety backstop bc something is already too close
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

        For each valid range r at angle theta closer than self.influence_radius
        add a force pointing from the obstacle back toward the robot in
        direction (-cos(theta), -sin(theta)) with magnitude that grows as
        r shrinks (e.g. k_repulsive * (1/r - 1/influence_radius)).
        """
        net_x, net_y = self.k_attractive, 0.0  # forward pull 
        for i, r in emnumerate(msg.ranges):
            if r >= self.influence_radius or r == 0.0:
                continue
            theta = msg.angle_min + i * msg.angle_increment
            magnitude = self.k_repulsive * (1.0 / r - 1.0 / self.influence_radius)
            net_x += magnitude * (-math.cos(theta))
            net_y += magnitude * (-math.sin(theta))
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
