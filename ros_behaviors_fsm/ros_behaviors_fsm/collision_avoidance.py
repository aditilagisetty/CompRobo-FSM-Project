"""
Collision Avoidance (skeleton)
--------
Combines the two day 3 e-stop approaches (bump-triggered, in
emergency_stop.py, and proximity-triggered, in distance_emergency_stop.py)
into a single behavior node. Fill in the TODOs to decide how the two
conditions combine, and add any additional conditions (e.g., side-swipe
trajectory) called for by the assignment.
"""
import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.stop_distance = 0.3
        self.bumped = False
        self.too_close = False
        self.create_timer(0.1, self.run_loop)

    def process_bump(self, msg):
        self.bumped = msg.left_front or msg.left_side or msg.right_front or msg.right_side

    def process_scan(self, msg):
        front_range = msg.ranges[0]
        # TODO: consider more than a single range reading for robustness
        self.too_close = 0.0 < front_range < self.stop_distance

    def run_loop(self):
        # TODO: decide what "stopped" should mean here -- full stop,
        # backing away, or something else -- and whether collision
        # avoidance should be able to un-stop itself.
        vel = Twist()
        if not self.bumped and not self.too_close:
            vel.linear.x = 0.1
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
