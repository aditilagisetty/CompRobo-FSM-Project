"""
Steer around obstacles and hard-stop when something gets too close.

Combines what used to be two separate behaviors: reactive stopping
(bump-triggered / very-close-range triggered), and continuous
steering around obstacles (potential fields) so the robot keeps
moving and reroutes, rather than just halting, for anything that
isn't an immediate emergency.
"""

import math
import time

from geometry_msgs.msg import Point, Twist
from neato2_interfaces.msg import Bump
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class CollisionAvoidance(Node):
    """
    Steer around obstacles with a potential field.

    Hard-stops for anything (or a bump) that's already too close for
    steering to help.
    """

    def __init__(self):
        """
        Set up subscriptions, publishers, and tuning constants.

        Sets up the bump/scan subscriptions, the cmd_vel and
        RViz-marker publishers, and the hard-stop/potential-field
        tuning constants.
        """
        super().__init__('collision_avoidance')
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_collision_avoidance', 10)
        self.marker_pub = self.create_publisher(Marker, 'collision_avoidance_force', 10)

        self.stop_distance = 0.3  # hard-stop trigger in meters, straight ahead
        # the potential field only ever pushes the robot forward, so an obstacle
        # off to the side can end up this close before the field steers away from
        # it -- back that up with a tighter hard-stop over a wider cone
        self.side_stop_distance = 0.22
        self.side_cone_deg = 45
        self.stop_distance_clear = 0.4
        self.side_stop_distance_clear = 0.32
        self.bumped = False
        self.too_close = False
        self.bump_timeout_sec = 0.3
        self.last_bump_time = None

        self.forward_speed = 0.1
        self.influence_radius = 1.0  # meters -- obstacles farther than this are ignored
        self.k_attractive = 1.0  # TODO: tune
        # the repulsion is summed over every ray while the pull is one
        # constant so this has to be small.
        self.k_repulsive = 0.02
        # P gain on heading error
        self.k_steer = 1.5
        self.max_angular_speed = 1.0  # rad/s
        # rotates every repulsive force vector by this much so a dead ahead
        # obstacle doesn't produce a net force that sits exactly on the
        # atan2 +/-pi discontinuity always
        # turns left when an obstacle is straight ahead
        self.repulsion_bias_angle = math.radians(20)

    # Ray i points i * msg.angle_increment counterclockwise from the robot's
    # front. the simulator's header says -pi, but the
    # lidar is mounted rotated 180 degrees, so ray 0 is the front - checked on
    # the recorded bags
    def _range_at_angle(self, msg, degrees):
        """
        Look up the scan range closest to the given degrees from the front.

        0 is straight ahead, 90 is facing left, and -90 is right.
        Returns inf for missing readings.
        """
        angle_rad = math.radians(degrees)
        # back calculate to find range index
        # wrap at one full turn
        rays_per_turn = round(2 * math.pi / msg.angle_increment)
        index = int(round(angle_rad / msg.angle_increment)) % rays_per_turn
        r = msg.ranges[index]
        # return of 0 means there is no obsticle detected rather than
        # there is an obstacle at distance 0 so return inf
        return r if r > 0.0 else float('inf')

    # do the _range_at_angle function for a large spread of angles
    def _min_range_in_cone(self, msg, center_deg, half_width_deg):
        """
        Return the smallest valid range within the cone around center_deg.

        Within plus or minus half_width_deg, so a dropped reading
        can't hide an obstacle.
        """
        readings = [
            self._range_at_angle(msg, center_deg + offset)
            for offset in range(-half_width_deg, half_width_deg + 1)
        ]
        return min(readings)

    def process_bump(self, msg):
        """
        Set self.bumped True on any real bump message.

        check_bump_timeout is what clears it, since the simulator
        never sends an explicit "bump cleared" message.
        """
        if msg.left_front or msg.left_side or msg.right_front or msg.right_side:
            self.last_bump_time = time.monotonic()
            self.bumped = True

    def check_bump_timeout(self):
        """Clear self.bumped once bump_timeout_sec has passed with no new bump."""
        if (
            self.bumped
            and time.monotonic() - self.last_bump_time > self.bump_timeout_sec
        ):
            self.bumped = False

    # decides whether to hard stop or steer around an obstacle
    def process_scan(self, msg):
        """
        Hard-stop if bumped or too close; otherwise steer with the potential field.

        Uses hysteresis on the release distance.
        """
        self.check_bump_timeout()
        self.front_range = self._min_range_in_cone(msg, center_deg=0, half_width_deg=10)
        self.side_range = self._min_range_in_cone(
            msg, center_deg=0, half_width_deg=self.side_cone_deg
        )
        # once too_close is set, it takes the larger *_clear
        # distance to release it, not just re-crossing the trigger distance
        if self.too_close:
            self.too_close = (
                self.front_range < self.stop_distance_clear
                or self.side_range < self.side_stop_distance_clear
            )
        else:
            self.too_close = (
                self.front_range < self.stop_distance
                or self.side_range < self.side_stop_distance
            )

        if self.bumped or self.too_close:
            # Hard-stop safety backstop bc something is already too close
            # for steering around it to make sense.
            self.vel_pub.publish(Twist())
            return

        net_x, net_y = self.compute_potential_field(msg)

        desired_heading = math.atan2(net_y, net_x)  # force vector into angle
        vel = Twist()
        # proportional control -- self.k_steer * desired_heading
        # bigger angle needed to turn --> harder we need to turn
        # clamp so its not commanded to turn harder than it can
        vel.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, self.k_steer * desired_heading),
        )
        # forward speed should slowdown based on total strength of repulsion too
        # repulsion_magnitude is large whenever anything is close in any direction
        # slowdown inversely proportional to repulsion
        # subtract the constant forward pull so open space isn't counted as repulsion
        repulsion_magnitude = math.hypot(net_x - self.k_attractive, net_y)
        slowdown = 1.0 / (1.0 + repulsion_magnitude)
        vel.linear.x = self.forward_speed * slowdown if net_x > 0 else 0.0
        self.vel_pub.publish(vel)

        self.publish_force_marker(net_x, net_y)

    def compute_potential_field(self, msg):
        """
        Return the net (x, y) force in the robot's frame.

        A constant forward attractive pull plus a repulsive
        contribution from every scan reading within
        self.influence_radius. For each valid range r at angle theta
        closer than self.influence_radius, adds a force pointing from
        the obstacle back toward the robot, rotated by
        self.repulsion_bias_angle, with magnitude that grows as r
        shrinks.
        """
        net_x, net_y = self.k_attractive, 0.0  # forward pull
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0 or r >= self.influence_radius:
                continue

            theta = i * msg.angle_increment  # angle from the front, see _range_at_angle
            magnitude = self.k_repulsive * (1.0 / r - 1.0 / self.influence_radius)
            # points from the obstacle back toward the robot (theta + pi)
            repulsion_angle = theta + math.pi + self.repulsion_bias_angle

            net_x += magnitude * math.cos(repulsion_angle)
            net_y += magnitude * math.sin(repulsion_angle)

        return net_x, net_y

    def publish_force_marker(self, x, y):
        """
        Visualize the net force vector as an arrow from the robot's origin.

        Published in RViz.
        """
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # shaft diameter
        marker.scale.y = 0.1  # head diameter
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=x, y=y, z=0.0)]
        self.marker_pub.publish(marker)


def main(args=None):
    """Initialize rclpy, spin the node until interrupted, then shut down."""
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# TODO: after food 1) re build all the code and run again to debug
