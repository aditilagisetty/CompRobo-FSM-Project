import math
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from neato2_interfaces.msg import Bump


class State(Enum):
    DRIVE_SQUARE = auto()
    COLLISION_AVOIDANCE = auto()
    WALL_FOLLOWING = auto()
    PATH_FOLLOWING = auto()


class FiniteStateController(Node):
    def __init__(self):
        super().__init__("finite_state_controller")
        self.state = State.DRIVE_SQUARE
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Bump, "bump", self.process_bump, 10)
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.create_subscription(
            String, "path_following_status", self.process_path_following_status, 10
        )
        self.create_timer(0.1, self.run_loop)

        self.stop_distance = 0.3  # collision-avoidance trigger, in meters
        self.wall_detect_distance = 0.75  # wall-following trigger, in meters

        self.bumped = False
        self.bump_timeout_sec = 0.3
        self.last_bump_time = None
        self.front_range = float("inf")
        self.obstacle_detected = False
        self.wall_detected = False
        self.left_clearance = float("inf")
        self.right_clearance = float("inf")

        self.wall_target_distance = 0.4  # meters to hold from the wall
        self.wall_kp_distance = 1.0  # TODO: tune
        self.wall_kp_align = 1.0  # TODO: tune
        self.wall_max_angular = 0.5  # rad/s cap
        self.wall_forward_speed = 0.1
        self.wall_side_dist = float("inf")
        self.wall_front_dist = float("inf")
        self.wall_rear_dist = float("inf")
        # +1 while following a wall on the robot's left, -1 on the right,
        # None when not currently following a wall. Latched in process_scan
        # so we don't flip sides mid-behavior if both sides briefly qualify.
        self.follow_side = None

        # True whenever the separate path_following.py node is actively
        # driving toward a goal
        self.path_following_active = False

        # drive_square state -- same time-based approach as
        # drive_square_single_threaded.py, adapted to fit this node's
        # existing per-tick run_loop instead of spinning up its own thread.
        self.square_side_length = 1.0  # meters (1m x 1m square)
        self.square_time_per_side = 5.0  # seconds
        self.square_time_per_turn = 2.0  # seconds
        self.square_executing_turn = False
        self.square_turns_executed = 0
        self.square_segment_start = None

    def process_bump(self, msg):
        if msg.left_front or msg.left_side or msg.right_front or msg.right_side:
            self.last_bump_time = time.monotonic()
            self.bumped = True

    def check_bump_timeout(self):
        if self.bumped and time.monotonic() - self.last_bump_time > self.bump_timeout_sec:
            self.bumped = False

    def process_path_following_status(self, msg):
        self.path_following_active = msg.data in ("following", "paused")

    def _range_at_angle(self, msg, degrees):
        """Look up the scan range closest to `degrees` from the robot's
        forward direction (0 = straight ahead, 90 = left, -90 = right).
        Returns inf for missing/zero ("no return") readings.
        """
        # Ray i points i * angle_increment counterclockwise from the front. Do not
        # use msg.angle_min bc the sim's header says -pi, but the lidar is mounted
        # rotated 180 degrees, so ray 0 is the front. Wrap at one full turn
        # not len(ranges) the scan has 361 the last repeating ray 0
        angle_rad = math.radians(degrees)
        rays_per_turn = round(2 * math.pi / msg.angle_increment)
        index = int(round(angle_rad / msg.angle_increment)) % rays_per_turn
        r = msg.ranges[index]
        return r if r > 0.0 else float("inf")

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
        """
        Process a LaserScan message to update the robot's state regarding obstacles and walls.
        """
        self.front_range = self._min_range_in_cone(msg, center_deg=0, half_width_deg=10)
        self.obstacle_detected = self.front_range < self.stop_distance
        self.left_clearance = self._range_at_angle(msg, 45)
        self.right_clearance = self._range_at_angle(msg, -45)

        left_range = self._range_at_angle(msg, 90)
        right_range = self._range_at_angle(msg, -90)
        wall_on_left = left_range < self.wall_detect_distance
        wall_on_right = right_range < self.wall_detect_distance
        self.wall_detected = wall_on_left or wall_on_right

        # we want to latch onto a side we are tracking so the process scan
        # and wall following  look at a consistient side instead of
        # jumping around every scan
        # MAKE SURE WALL FOLLOWING AGREES WITH THIS
        if self.wall_detected and self.follow_side is None:
            self.follow_side = 1 if wall_on_left else -1
        elif not self.wall_detected:
            self.follow_side = None

        if self.follow_side is not None:
            self.wall_side_dist = self._range_at_angle(msg, 90 * self.follow_side)
            self.wall_front_dist = self._range_at_angle(msg, 45 * self.follow_side)
            self.wall_rear_dist = self._range_at_angle(msg, 135 * self.follow_side)

    def run_loop(self):
        self.check_bump_timeout()
        previous_state = self.state

        if self.path_following_active:
            self.state = State.PATH_FOLLOWING
        elif self.state == State.PATH_FOLLOWING:
            # path_following.py just went idle resume normal FSM
            # behavior starting from DRIVE_SQUARE
            self.state = State.DRIVE_SQUARE
        elif self.state == State.DRIVE_SQUARE:
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

        if self.state == State.DRIVE_SQUARE and previous_state != State.DRIVE_SQUARE:
            # Restart timing on whichever segment we were on rather than
            # counting the paused time (spent in another state) against it.
            self.square_segment_start = None

        if self.state == State.DRIVE_SQUARE:
            self.handle_drive_square()
        elif self.state == State.COLLISION_AVOIDANCE:
            self.handle_collision_avoidance()
        elif self.state == State.WALL_FOLLOWING:
            self.handle_wall_following()
        elif self.state == State.PATH_FOLLOWING:
            self.handle_path_following()

    def handle_path_following(self):
        """Intentionally does nothing. path_following.py is a separate node
        with its own cmd_vel publisher and its own built-in bump/estop/
        obstacle-close pause logic -- while it's active, this node
        deliberately publishes nothing at all, rather than also reacting to
        the same sensors and fighting path_following.py over cmd_vel (two
        nodes publishing conflicting Twist messages at once). This is the
        "chain parallel nodes together" FSM strategy: path_following.py
        keeps running exactly as it does standalone, and this state machine
        just tracks whether it's currently in control and steps aside.
        """
        pass

    def handle_drive_square(self):
        """Drives a 1m x 1m square, using the same time-based approach as
        drive_square_single_threaded.py. Adapted to a per-tick style (rather
        than a dedicated Thread, as in drive_square.py) since that's what
        this node's run_loop already is -- letting collision-avoidance and
        wall-following interrupt/resume this state doesn't require any
        extra thread coordination this way.
        """
        if self.square_turns_executed >= 4:
            self.vel_pub.publish(Twist())
            return

        if self.square_segment_start is None:
            self.square_segment_start = self.get_clock().now()

        duration = (
            self.square_time_per_turn
            if self.square_executing_turn
            else self.square_time_per_side
        )
        elapsed = self.get_clock().now() - self.square_segment_start

        msg = Twist()
        if elapsed > rclpy.time.Duration(seconds=duration):
            if self.square_executing_turn:
                self.square_turns_executed += 1
            self.square_executing_turn = not self.square_executing_turn
            self.square_segment_start = None
            # leave msg as zero velocity so we stop briefly between segments
        elif self.square_executing_turn:
            msg.angular.z = (math.pi / 2) / duration
        else:
            msg.linear.x = self.square_side_length / duration
        self.vel_pub.publish(msg)

    def handle_collision_avoidance(self):
        """Backs away from whatever triggered the stop while turning toward
        whichever side has more space so the robot reroutes
        """
        vel = Twist()
        vel.linear.x = -0.05  # m/s
        turn_speed = 0.3  # rad/s
        vel.angular.z = (
            turn_speed if self.left_clearance > self.right_clearance else -turn_speed
        )
        self.vel_pub.publish(vel)

    # if we are farther than .4 m from the wall, distance error is positive and robot turns towards it
    # follow side only resets when no wall is detected on either side
    def handle_wall_following(self):
        vel = Twist()
        vel.linear.x = self.wall_forward_speed
        side = self.follow_side
        if side is not None:
            if math.isinf(self.wall_side_dist):
                distance_error = 0.0
            else:
                distance_error = self.wall_side_dist - self.wall_target_distance
            if math.isinf(self.wall_front_dist) or math.isinf(self.wall_rear_dist):
                align_error = 0.0
            else:
                align_error = self.wall_front_dist - self.wall_rear_dist
            turn = side * (
                self.wall_kp_distance * distance_error
                + self.wall_kp_align * align_error
            )
            vel.angular.z = max(
                -self.wall_max_angular, min(self.wall_max_angular, turn)
            )
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
