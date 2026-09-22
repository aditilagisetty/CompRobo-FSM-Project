import math
import select
import sys
import termios
import tty
from collections import deque
from threading import Thread

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from .angle_helpers import euler_from_quaternion
from .room_map import DEFAULT_MAP_FILE, RoomMap

HELP = """
  w / s   forward / backward        q / e   curve left / right
  a / d   turn left / right         space   stop
  + / -   speed up / down           m       save map      Ctrl-C  save + quit
"""


class TeleopScan(Node):
    """Node that implements a simple teleoperation interface
    for controlling the robot using keyboard input.
    The robot can be driven forward, backward, and turned left or right.
    The robot's movements are also recorded to create
    a map of the environment using laser scan data.
    """

    KEY_BINDINGS = {
        "w": (1.0, 0.0),
        "s": (-1.0, 0.0),
        "a": (0.0, 1.0),
        "d": (0.0, -1.0),
        "q": (1.0, 0.5),
        "e": (1.0, -0.5),
    }

    def __init__(self):
        super().__init__("teleop_scan")
        self.declare_parameter("map_file", DEFAULT_MAP_FILE)
        self.declare_parameter("map_size", 20.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("angular_speed", 0.6)
        self.declare_parameter("lidar_offset_x", -0.084)
        self.map_file = self.get_parameter("map_file").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.lidar_offset_x = self.get_parameter("lidar_offset_x").value

        self.room_map = RoomMap(
            self.get_parameter("map_size").value, self.get_parameter("resolution").value
        )
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.odom_history = deque(maxlen=100)
        self.spin_rate = 0.0
        self.scans_used = 0
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0

        self.vel_pub = self.create_publisher(Twist, "cmd_vel_teleop_scan", 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "room_map", map_qos)
        self.create_subscription(Odometry, "odom", self.process_odom, 10)
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.create_timer(0.1, self.publish_velocity)
        self.create_timer(2.0, self.publish_map)
        self.create_timer(5.0, self.report_status)

    def process_odom(self, msg):
        """Processes the incoming Odometry message to update the robot's position
        and orientation. The odometry data is used to track the
        robot's movement and build a map of the environment.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.spin_rate = abs(msg.twist.twist.angular.z)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_history.append((stamp, self.x, self.y, self.yaw))
        self.have_odom = True

    def pose_at(self, stamp):
        """Returns the robot's pose (x, y, yaw) at the given
        timestamp by interpolating between the two closestodometry readings.
        If the requested timestamp is outside the range of recorded odometry data, None is returned.
        """
        history = list(self.odom_history)
        if len(history) < 2 or stamp < history[0][0]:
            return None
        for i in range(len(history) - 1, 0, -1):
            t0, x0, y0, a0 = history[i - 1]
            t1, x1, y1, a1 = history[i]
            if t0 <= stamp:
                f = min(1.0, (stamp - t0) / (t1 - t0)) if t1 > t0 else 1.0
                da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
                return x0 + f * (x1 - x0), y0 + f * (y1 - y0), a0 + f * da

    def process_scan(self, msg):
        """Processes the incoming LaserScan message to update the
        robot's map of the environment. The laser scan data is used to detect obstacles and walls,
          which are then added to the occupancy grid map."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.pose_at(stamp)
        # a fast spin smears the map, odom yaw is too rough for it
        if pose is None or self.spin_rate > 0.4:
            return
        self.room_map.add_scan(
            pose[0],
            pose[1],
            pose[2],
            msg.ranges,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
            self.lidar_offset_x,
        )
        self.scans_used += 1

    def handle_key(self, key):
        """Handles keyboard input to control the robot's movement and behavior.
        The robot can be driven forward, backward, and turned left or right using the WASD keys.
        Additional commands allow for speed adjustments, stopping the robot,
        saving the map, and switching between different states of operation."""
        key = key.lower()
        if key in self.KEY_BINDINGS:
            lin, ang = self.KEY_BINDINGS[key]
            self.linear_cmd = lin * self.linear_speed
            self.angular_cmd = ang * self.angular_speed
        elif key in (" ", "x"):
            self.linear_cmd = 0.0
            self.angular_cmd = 0.0
        elif key in ("+", "="):
            self.scale_speeds(1.1)
        elif key in ("-", "_"):
            self.scale_speeds(0.9)
        elif key == "m":
            self.save_map()

    def scale_speeds(self, factor):
        """Scales the robot's linear and angular speeds by the given factor.
        This allows for dynamic adjustment of the robot's speed during operation."""
        self.linear_speed *= factor
        self.angular_speed *= factor
        self.linear_cmd *= factor
        self.angular_cmd *= factor
        self.get_logger().info(
            f"speed: {self.linear_speed:.2f} m/s, {self.angular_speed:.2f} rad/s"
        )

    def publish_velocity(self):
        """Publishes the current linear and angular velocity commands to the robot.
        This method is called periodically to ensure that the robot
        continues to move according to the latest commands received from keyboard input.
        """
        self.drive(self.linear_cmd, self.angular_cmd)

    def drive(self, linear, angular):
        """Publishes a Twist message with the given linear and angular velocities
        to the robot's velocity command topic."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)

    def stop(self):
        """Stops the robot by publishing a zero velocity command."""
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.drive(0.0, 0.0)

    def publish_map(self):
        """Publishes the current occupancy grid map to the "room_map" topic.
        This method is called periodically to provide an updated view of the
        environment based on the latest laser scan data."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.info.resolution = self.room_map.resolution
        msg.info.width = self.room_map.size
        msg.info.height = self.room_map.size
        msg.info.origin.position.x = self.room_map.origin_x
        msg.info.origin.position.y = self.room_map.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.room_map.to_occupancy_data()
        self.map_pub.publish(msg)

    def save_map(self):
        """Saves the current occupancy grid map to a file.
        The map is saved in a format that can be loaded later
        for further analysis or use in navigation tasks."""
        path = self.room_map.save(self.map_file)
        self.get_logger().info(f"saved map to {path}")

    def report_status(self):
        """Reports the current status of the robot, including its position,
         orientation, and the number of laser scans used to build the map.
        This method is called periodically to provide feedback on the robot's operation.
        """
        self.get_logger().info(
            f"pose=({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.0f}deg) "
            f"scans mapped={self.scans_used}"
        )


def keyboard_loop(node):
    """Runs a loop that listens for keyboard input and passes it to the given
    TeleopScan node for processing.
    The loop runs in a separate thread to allow for non-blocking operation
    of the robot while still responding to user input."""
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                node.handle_key(sys.stdin.read(1))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)


def main(args=None):
    """
    Main function to initialize the ROS2 node and start spinning it.
    This function sets up the TeleopScan node and keeps it running until the program is terminated.
    It also handles keyboard input for controlling the robot and saving the map."""
    # let Ctrl-C raise here so the robot still gets a zero velocity before rclpy shuts down
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopScan()
    if not sys.stdin.isatty():
        node.get_logger().error(
            "teleop_scan needs to be run from a terminal " "(stdin is not a tty)"
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    print(HELP)
    print(f"Map will be saved to {node.map_file}\n")
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    try:
        keyboard_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.save_map()
        rclpy.shutdown()
        spin_thread.join()
        node.destroy_node()


if __name__ == "__main__":
    main()
