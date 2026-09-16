"""
Obstacle Avoider (skeleton) -- self-designed behavior
--------
This is meant to be the self-designed behavior satisfying the assignment's
"at least one behavior outside class activities" requirement -- the actual
avoidance logic below is left as TODOs for you to design and implement.

Objective: move forward while reactively steering *around* obstacles --
unlike collision_avoidance, this shouldn't stop, it should adjust heading
and keep going.

Suggested approach: potential fields. A constant "attractive" force always
pulls the robot forward; every nearby laser range reading exerts a
"repulsive" force pointing away from that obstacle, growing stronger as the
obstacle gets closer. Summing all of these into one net force vector gives
a desired direction of travel -- steer toward it with a proportional
controller, and slow down when the net repulsion is strong.

Simpler alternative (closer to the assignment's "basic" version): when
something's found close ahead, turn ~90 degrees away from it, then turn back
toward your original heading once it's no longer in the way.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Optional per the assignment, but useful for seeing what the robot
        # "thinks" in RViz: Add -> By topic -> obstacle_avoider_force -> Marker.
        self.marker_pub = self.create_publisher(Marker, 'obstacle_avoider_force', 10)

        self.forward_speed = 0.1
        self.influence_radius = 1.0  # meters; obstacles farther than this are ignored
        self.k_attractive = 1.0      # TODO: tune
        self.k_repulsive = 1.0       # TODO: tune

    def process_scan(self, msg):
        net_x, net_y = self.compute_potential_field(msg)

        # TODO: convert the net (net_x, net_y) force vector -- in the
        # robot's own frame, x = forward, y = left -- into an actual drive
        # command. Roughly: desired_heading = atan2(net_y, net_x); steer
        # proportionally toward it (angular.z), and drive forward at a
        # speed related to net force magnitude (e.g. slow down or stop if
        # net_x is small/negative, meaning something is pushing back hard).
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
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
