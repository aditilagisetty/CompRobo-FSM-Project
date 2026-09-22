"""
Pilot the robot in a square with time-based, estop-aware driving.

This node implements a simple time-based approach to driving the
robot in a square. The system makes use of a special ``estop`` topic
that can trigger the robot to automatically stop when true is
received on that topic.
"""

import math
from threading import Thread
from time import sleep

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .angle_helpers import euler_from_quaternion


class DrawSquare(Node):
    """A class that implements a node to pilot a robot in a square."""

    def __init__(self):
        """
        Set up subscriptions and start the first drive on its own thread.

        Sets up the estop/scan/odom subscriptions and the
        current_mode subscription that re-triggers the square under
        the gateway FSM.
        """
        super().__init__('draw_square_with_estop')
        self.manual_estop = False
        self.obstacle_close = False
        self.stop_distance = 0.5
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_drive_square', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)
        self.create_subscription(String, 'current_mode', self.process_current_mode, 10)
        self._last_mode = None
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()

    def process_current_mode(self, msg):
        """
        Re-run the square whenever the gateway FSM freshly enters DRIVE SQUARE.

        run_loop only drives it once per thread and otherwise
        wouldn't do anything the next time that state is entered.
        """
        entered_drive_square = (
            msg.data == 'DRIVE SQUARE' and self._last_mode != 'DRIVE SQUARE'
        )
        self._last_mode = msg.data
        if entered_drive_square and not self.run_loop_thread.is_alive():
            self.run_loop_thread = Thread(target=self.run_loop)
            self.run_loop_thread.start()

    def stopped(self):
        """
        Return whether the robot should currently be halted.

        From either the manual estop topic or a nearby obstacle.
        """
        return self.manual_estop or self.obstacle_close

    def process_odom(self, msg):
        """
        Track the robot's current position and yaw from odometry.

        So turn_left/drive_forward can measure real motion instead
        of assuming a speed.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        _, _, self.current_yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    @staticmethod
    def _yaw_turned(current_yaw, start_yaw):
        """Signed rotation from start_yaw to current_yaw, in (-pi, pi]."""
        diff = current_yaw - start_yaw
        return math.atan2(math.sin(diff), math.cos(diff))

    def handle_estop(self, msg):
        """
        Handle messages received on the estop topic.

        msg (std_msgs.msg.Bool) takes value true if we estop and
        false otherwise.
        """
        self.manual_estop = bool(msg.data)
        if self.manual_estop:
            self.drive(linear=0.0, angular=0.0)

    def process_scan(self, msg):
        """
        Handle laser scan data, triggering the same stop condition as estop.

        Triggers if something is within stop_distance in front of
        the robot, and clears it once the obstacle is no longer
        close. msg is the current sensor_msgs.msg.LaserScan.
        """
        front_range = msg.ranges[0]
        was_close = self.obstacle_close
        self.obstacle_close = 0.0 < front_range < self.stop_distance
        if self.obstacle_close and not was_close:
            print(f'Obstacle detected {front_range:.2f}m ahead, stopping!')
            self.drive(linear=0.0, angular=0.0)

    def run_loop(self):
        """
        Execute the main logic for driving the square.

        Does not return until the square is finished or the estop is
        pressed.
        """
        # the first message on the publisher is often missed
        self.drive(0.0, 0.0)
        sleep(1)
        for _ in range(4):
            if not self.stopped():
                print('driving forward')
                self.drive_forward(1.0)
            if not self.stopped():
                print('turning left')
                self.turn_left()
        print('done with run loop')

    def drive(self, linear, angular):
        """
        Drive with the specified linear and angular velocity.

        linear is in m/s, angular is in radians/s.
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def settle(self, duration=1.0):
        """
        Command zero velocity and wait for residual motion to die down.

        Waits for any residual motion (from deceleration lag) to
        fully die down before switching to a different kind of
        motion. Without this, turn_left can start commanding rotation
        while the robot is still physically coasting forward from the
        previous drive_forward (and vice versa), turning a clean
        stop-then-pivot into an arc.
        """
        self.drive(linear=0.0, angular=0.0)
        sleep(duration)

    def turn_left(self):
        """
        Turn left 90 degrees using proportional control on the remaining angle.

        So the robot is already slowing down as it approaches the
        target instead of coasting past it under momentum after a
        hard stop command.
        """
        max_angular_vel = 0.2  # lowered from 0.3 to shrink the coast-to-stop distance
        min_angular_vel = 0.05  # floor so the final approach doesn't stall
        tolerance = 0.02  # ~1 degree; close enough to call it done
        k_p = 1.0
        target_angle = math.pi / 2

        if self.stopped():
            return
        start_yaw = self.current_yaw
        stopped_early = False
        while not self.stopped():
            remaining = target_angle - self._yaw_turned(self.current_yaw, start_yaw)
            if remaining <= tolerance:
                break
            speed = max(min_angular_vel, min(max_angular_vel, k_p * remaining))
            self.drive(linear=0.0, angular=speed)
            sleep(0.05)
        else:
            stopped_early = True
        if not self.stopped():
            self.settle()
        else:
            self.drive(linear=0.0, angular=0.0)
        actual_deg = math.degrees(self._yaw_turned(self.current_yaw, start_yaw))
        print(
            f'turn_left done: target=90.0deg actual={actual_deg:.1f}deg '
            f'error={actual_deg - 90.0:+.1f}deg stopped_early={stopped_early}'
        )

    def drive_forward(self, distance):
        """
        Drive straight until odometry reports the given distance is covered.

        Uses proportional control on the remaining distance so the
        robot is already slowing down as it approaches the target
        instead of coasting past it under momentum after a hard stop.
        distance must be positive.
        """
        max_linear_vel = 0.1  # same top speed as before
        min_linear_vel = 0.02  # floor so the final approach doesn't stall
        tolerance = 0.02  # meters; close enough to call it done
        # Lower than turn_left's k_p: this sim's linear deceleration is
        # weaker relative to the top speed than its angular deceleration is,
        # so k_p=1.0 here only starts slowing down in the last 10cm -- not
        # enough runway to actually shed speed before reaching the target.
        # k_p=0.5 starts slowing twice as early and removes most of the
        # overshoot (simulated: ~4.5cm -> ~0.85cm on a 1m leg).
        k_p = 0.5

        if self.stopped():
            return
        start_x = self.current_x
        start_y = self.current_y
        stopped_early = False
        while not self.stopped():
            traveled = math.hypot(self.current_x - start_x, self.current_y - start_y)
            remaining = distance - traveled
            if remaining <= tolerance:
                break
            speed = max(min_linear_vel, min(max_linear_vel, k_p * remaining))
            self.drive(linear=speed, angular=0.0)
            sleep(0.05)
        else:
            stopped_early = True
        if not self.stopped():
            self.settle()
        else:
            self.drive(linear=0.0, angular=0.0)
        actual_traveled = math.hypot(self.current_x - start_x, self.current_y - start_y)
        print(
            f'drive_forward done: target={distance:.2f}m actual={actual_traveled:.2f}m '
            f'error={actual_traveled - distance:+.2f}m stopped_early={stopped_early}'
        )


def main(args=None):
    """Initialize rclpy, spin the node until interrupted, then shut down."""
    rclpy.init(args=args)
    node = DrawSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
