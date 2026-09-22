import math
import select
import sys
import termios
import threading
import time
import tty
import rclpy
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class FSMNode(Node):
    """
    Finite State Machine (FSM) Node for Robot Control"""

    def __init__(self):
        super().__init__("fsm_node")

        self.state = "WALL FOLLOW"  # Initial state

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.mode_pub = self.create_publisher(String, "current_mode", 10)

        # Subscription for different nodes to send velocity commands
        self.create_subscription(Twist, "cmd_vel_wall_follower", self.wall_follower, 10)
        self.create_subscription(
            Twist, "cmd_vel_collision_avoidance", self.obstacle_avoidance, 10
        )
        self.create_subscription(Twist, "cmd_vel_teleop_scan", self.teleop_scan, 10)

        self.create_subscription(
            Twist, "cmd_vel_path_following", self.path_following, 10
        )

        self.create_subscription(Twist, "cmd_vel_drive_square", self.drive_square, 10)

        # Subscrption for LaserScan data
        self.create_subscription(LaserScan, "scan", self.run_loop, 10)

        # Same t/m/g/p switches as the keyboard listener below, but over a
        # topic instead of raw stdin -- this works under ros2 launch (or any
        # other non-interactive process), where stdin isn't a real terminal
        # and the keyboard listener can't run at all, e.g.:
        #   ros2 topic pub -1 /fsm_command std_msgs/String "data: p"
        self.create_subscription(String, "fsm_command", self.process_fsm_command, 10)

        # bump subscription
        self.create_subscription(Bump, "bump", self.process_bump, 10)
        self.bumped = False
        self.bump_timeout_sec = 0.3
        self.last_bump_time = None

        self.has_teleop_run = False  # Flag to check if teleop has run before

        # wall_follower turns away on its own once something is within
        # wall_follower_turn_distance -- that keeps front_distance hovering
        # just above obstacle_distance instead of ever crossing it, so
        # OBSTACLE AVOIDANCE was never reached even while stuck turning in
        # place at a corner. Back the absolute-distance switch up with a
        # timeout: if front_distance has stayed inside wall_follower's own
        # reactive zone for too long without fully clearing, hand off to
        # collision_avoidance anyway.
        self.obstacle_distance = 0.5
        self.wall_follow_recover_distance = 0.7
        self.wall_follower_turn_distance = 1.0
        self.wall_follower_clear_distance = 1.3
        self.stuck_timeout_sec = 3.0
        self.close_since = None

        # Starting separate thread for keyboard input
        self.key_thread = threading.Thread(target=self.keyboard_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

    def process_bump(self, msg):
        """sets self.bumped True on any real bump message.
        Clearing it is check_bump_timeout()'s job instead.
        """
        if msg.left_front or msg.left_side or msg.right_front or msg.right_side:
            self.last_bump_time = time.monotonic()
            self.bumped = True

    def check_bump_timeout(self):
        """Clears self.bumped once bump_timeout_sec has passed.
        """
        if self.bumped and time.monotonic() - self.last_bump_time > self.bump_timeout_sec:
            self.bumped = False

    def set_state(self, new_state):
        """
        Sets the current state of the FSM and publishes it to the "current_mode" topic.
        """
        self.state = new_state
        self.mode_pub.publish(String(data=new_state))

    def handle_key(self, key):
        """The t/m/g/p switch logic, shared by the keyboard listener and
        process_fsm_command so the two input paths can't drift apart.
        """
        key = key.lower()
        if key == "t":
            self.set_state("TELEOP SCAN")
        elif key == "m" and self.state == "TELEOP SCAN":
            self.set_state("DRIVE SQUARE")
        elif key == "g":
            self.set_state("WALL FOLLOW")
        elif key == "p" and self.has_teleop_run:
            self.set_state("PATH FOLLOWING")

    def process_fsm_command(self, msg):
        """Same switches as keyboard_listener, delivered over the
        fsm_command topic instead of raw stdin -- this is the one that
        still works under ros2 launch.
        """
        self.handle_key(msg.data.strip())

    def keyboard_listener(self):
        """Listens for raw key presses in terminal without pressing Enter."""
        if not sys.stdin.isatty():
            self.get_logger().error(
                "fsm_node keyboard control needs a real terminal (stdin is "
                "not a tty) -- publish to /fsm_command instead, e.g. "
                "ros2 topic pub -1 /fsm_command std_msgs/String \"data: p\""
            )
            return
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    self.handle_key(sys.stdin.read(1))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def wall_follower(self, msg):
        """Publishes velocity commands from the wall follower node if the FSM is in the "WALL FOLLOW" state."""
        if self.state == "WALL FOLLOW":
            self.vel_pub.publish(msg)

    def drive_square(self, msg):
        """Publishes velocity commands from the drive square node if the FSM is in the "DRIVE SQUARE" state."""
        if self.state == "DRIVE SQUARE":
            self.vel_pub.publish(msg)

    def obstacle_avoidance(self, msg):
        """Publishes velocity commands from the obstacle avoidance node if the FSM is in the "OBSTACLE AVOIDANCE" state."""
        if self.state == "OBSTACLE AVOIDANCE":
            self.vel_pub.publish(msg)

    def path_following(self, msg):
        """Publishes velocity commands from the path following node if the FSM is in the "PATH FOLLOWING" state."""
        if self.state == "PATH FOLLOWING":
            self.vel_pub.publish(msg)

    def teleop_scan(self, msg):
        """Publishes velocity commands from the teleop scan node if the FSM is in the "TELEOP SCAN" state."""
        if self.state == "TELEOP SCAN":
            self.vel_pub.publish(msg)
            self.has_teleop_run = True  # Set the flag to True when teleop_scan is run

    def run_loop(self, msg):
        """Main loop that checks the LaserScan data to determine if the robot should switch between "WALL FOLLOW" and "OBSTACLE AVOIDANCE" states."""
        if self.state == "TELEOP SCAN":
            return  # Skip processing if in TELEOP SCAN mode

        
        self.check_bump_timeout()

        # Check for obstacles in front of the robot
        front_cone = msg.ranges[:10] + msg.ranges[-10:]
        valid_ranges = [r for r in front_cone if math.isfinite(r) and r > 0.0]
        front_distance = min(valid_ranges) if valid_ranges else float("inf")

        # print the current state for debugging purpose
        print(self.state)

        # tracked independent of state, so a brief dip into OBSTACLE AVOIDANCE
        # and back doesn't reset the clock on a corner we're still stuck at
        now = self.get_clock().now().nanoseconds / 1e9
        if front_distance < self.wall_follower_turn_distance:
            if self.close_since is None:
                self.close_since = now
        elif front_distance > self.wall_follower_clear_distance:
            self.close_since = None
        stuck = (
            self.close_since is not None
            and now - self.close_since > self.stuck_timeout_sec
        )

        if self.state == "WALL FOLLOW":
            if front_distance < self.obstacle_distance or stuck or self.bumped:
                self.set_state("OBSTACLE AVOIDANCE")
        elif self.state == "OBSTACLE AVOIDANCE":
            # stay in OBSTACLE AVOIDANCE while still stuck or bumped even if
            # a nudge from the potential field briefly pushed front back out
            if (front_distance > self.wall_follow_recover_distance
                    and not stuck and not self.bumped):
                self.set_state("WALL FOLLOW")


def main(args=None):
    rclpy.init(args=args)
    fsm_node = FSMNode()
    rclpy.spin(fsm_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
