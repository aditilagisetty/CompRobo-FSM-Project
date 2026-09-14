import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump


class State(Enum):
    DRIVE_SQUARE = auto()
    COLLISION_AVOIDANCE = auto()
    WALL_FOLLOWING = auto()


class FiniteStateController(Node):
    def __init__(self):
        super().__init__('finite_state_controller')
        self.state = State.DRIVE_SQUARE
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_timer(0.1, self.run_loop)

        self.stop_distance = 0.3          # collision-avoidance trigger, in meters
        self.wall_detect_distance = 0.75  # wall-following trigger, in meters

        self.bumped = False
        self.front_range = float('inf')
        self.obstacle_detected = False
        self.wall_detected = False
        # +1 while following a wall on the robot's left, -1 on the right,
        # None when not currently following a wall. Latched in process_scan
        # so we don't flip sides mid-behavior if both sides briefly qualify.
        self.follow_side = None

    def process_bump(self, msg):
        self.bumped = bool(msg.left_front or msg.left_side
                            or msg.right_front or msg.right_side)

    def _range_at_angle(self, msg, degrees):
        """Look up the scan range closest to `degrees` from the robot's
        forward direction (0 = straight ahead, 90 = left, -90 = right).
        Returns inf for missing/zero ("no return") readings.
        """
        angle_rad = math.radians(degrees)
        index = int(round((angle_rad - msg.angle_min) / msg.angle_increment))
        index %= len(msg.ranges)
        r = msg.ranges[index]
        return r if r > 0.0 else float('inf')

    def _min_range_in_cone(self, msg, center_deg, half_width_deg):
        """Smallest valid range within +/- half_width_deg of center_deg,
        used so a single dropped/zero reading can't hide an obstacle.
        """
        readings = [
            self._range_at_angle(msg, center_deg + offset)
            for offset in range(-half_width_deg, half_width_deg + 1)
        ]
        return min(readings)

    def process_scan(self, msg):
        self.front_range = self._min_range_in_cone(msg, center_deg=0, half_width_deg=10)
        self.obstacle_detected = self.front_range < self.stop_distance

        left_range = self._range_at_angle(msg, 90)
        right_range = self._range_at_angle(msg, -90)
        wall_on_left = left_range < self.wall_detect_distance
        wall_on_right = right_range < self.wall_detect_distance
        self.wall_detected = wall_on_left or wall_on_right

        # Latch which side we're tracking so process_scan (and
        # handle_wall_following, once you implement it) keep looking at a
        # consistent side instead of jumping around every scan.
        if self.wall_detected and self.follow_side is None:
            self.follow_side = 1 if wall_on_left else -1
        elif not self.wall_detected:
            self.follow_side = None

    def run_loop(self):
        if self.state == State.DRIVE_SQUARE:
            if self.bumped or self.obstacle_detected:
                self.state = State.COLLISION_AVOIDANCE
            elif self.wall_detected:
                self.state = State.WALL_FOLLOWING
        elif self.state == State.COLLISION_AVOIDANCE:
            if not self.bumped and not self.obstacle_detected:
                self.state = State.DRIVE_SQUARE
        elif self.state == State.WALL_FOLLOWING:
            if not self.wall_detected:
                self.state = State.DRIVE_SQUARE

        if self.state == State.DRIVE_SQUARE:
            self.handle_drive_square()
        elif self.state == State.COLLISION_AVOIDANCE:
            self.handle_collision_avoidance()
        elif self.state == State.WALL_FOLLOWING:
            self.handle_wall_following()

    def handle_drive_square(self):
        # TODO: implement or delegate to drive_square logic
        pass

    def handle_collision_avoidance(self):
        # Sensing is done -- this just stops the robot. TODO: consider
        # something more graceful (e.g., backing away or re-routing) if you
        # want collision avoidance to do more than e-stop.
        self.vel_pub.publish(Twist())

    def handle_wall_following(self):
        # self.follow_side (+1 = wall on the left, -1 = on the right) tells
        # you which side to track. TODO: implement the actual steering law
        # -- e.g., compare ranges at +/-45 deg and +/-135 deg on that side
        # (see wall_follower.py) to compute a proportional correction to
        # angular.z, then drive forward with that correction applied.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
