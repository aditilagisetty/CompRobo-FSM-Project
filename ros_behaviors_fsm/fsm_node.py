import math
import select
import sys
import termios
import threading
import tty
import rclpy
from geometry_msgs.msg import Twist
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

        self.has_teleop_run = False  # Flag to check if teleop has run before

        # Starting separate thread for keyboard input
        self.key_thread = threading.Thread(target=self.keyboard_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

    def set_state(self, new_state):
        """
        Sets the current state of the FSM and publishes it to the "current_mode" topic.
        """
        self.state = new_state
        self.mode_pub.publish(String(data=new_state))

    def keyboard_listener(self):
        """Listens for raw key presses in terminal without pressing Enter."""
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == "t":
                        self.set_state("TELEOP SCAN")
                    elif key.lower() == "m" and self.state == "TELEOP SCAN":
                        self.set_state("DRIVE SQUARE")
                    elif key.lower() == "g":
                        self.set_state("WALL FOLLOW")
                    elif key.lower() == "p" and self.has_teleop_run:
                        self.set_state("PATH FOLLOWING")

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

        # Check for obstacles in front of the robot
        front_cone = msg.ranges[:10] + msg.ranges[-10:]
        valid_ranges = [r for r in front_cone if math.isfinite(r) and r > 0.0]
        front_distance = min(valid_ranges) if valid_ranges else float("inf")

        # print the current state for debugging purpose
        print(self.state)

        if (
            front_distance < 0.5 and self.state == "WALL FOLLOW"
        ):  # If an obstacle is detected within 0.5 meters
            self.set_state("OBSTACLE AVOIDANCE")
        elif (
            front_distance > 0.7 and self.state == "OBSTACLE AVOIDANCE"
        ):  # leeway to prevent rapid switching
            self.set_state("WALL FOLLOW")


def main(args=None):
    rclpy.init(args=args)
    fsm_node = FSMNode()
    rclpy.spin(fsm_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
