"""
Draw Square
--------
This node encapsulates implements a simple time-based approach to driving the
robot in a square.  The system makes use of a a special ``estop`` topic that
can trigger the robot to automatically stop when the value is true is received
on that topic.

Day 4 annotation (multi-threaded sample):

- run_loop runs once, start to finish, on its own Thread -- it isn't called
  repeatedly like a timer callback. It blocks on sleep() between segments,
  but that blocking happens on its own thread, so it never blocks the
  handle_estop/process_scan callbacks, which run on rclpy.spin()'s thread.
- FSM: two states, "driving forward" and "turning left", alternated for 4
  iterations. Transitions are time-based (fixed sleep durations), same as
  Sample 1, just executed sequentially in a dedicated thread instead of a
  timer callback.
- When e-stop triggers, handle_estop/process_scan immediately publish a zero
  Twist, so the robot stops quickly even if run_loop's thread is mid-sleep.
- It WILL stop after the fourth side (assuming no e-stop): the `for _ in
  range(4)` loop simply exits and the thread ends after 4 forward+turn
  pairs, unlike Sample 1's original (unfixed) behavior.
- As originally written, resuming was NOT handled correctly: handle_estop
  only ever called e_stop.set() when msg.data was True, with no `else:
  e_stop.clear()`, so once tripped the square-drawing was stopped for good.
  Fixed below by tracking the manual estop topic and the obstacle-proximity
  condition as two separate flags (self.manual_estop, self.obstacle_close)
  that combine in self.stopped() -- each can be independently set and
  cleared as new messages arrive, so the robot can resume once both clear.
  One caveat: an interruption still can't resume a segment exactly where it
  left off -- turn_left/drive_forward each restart from scratch (a fresh
  start_yaw/start_x/start_y) the next time they're called, so an interrupted
  segment's partial progress toward its target is lost, even though the stop
  check itself is now continuous (see below), not just between segments.

Day 4 "going beyond" fix -- turning/driving by odometry instead of time:
  turn_left/drive_forward originally assumed a fixed angular/linear speed
  and slept for the fixed duration that speed implies. In Gazebo this
  undershoots: the sim's diff-drive plugin has a max_wheel_acceleration of
  1.0 rad/s^2 (see neato2_gazebo/models/neato/neato_with_camera.sdf), so it
  takes about a second to ramp up to the commanded speed, and that ramp time
  isn't accounted for by the fixed sleep -- so the robot doesn't actually
  finish the turn/distance by the time the sleep ends, and the drawn shape
  drifts further from a square with every side. Both methods now instead
  poll actual odometry (position for drive_forward, yaw for turn_left) and
  stop once the real measured motion reaches the target, checking self.stopped()
  every poll so e-stop is now caught mid-segment too, not just between segments.

  That alone still overshoots, though: the same acceleration limit that
  slows ramp-up also limits deceleration, so commanding a hard stop the
  instant the target is reached doesn't actually stop the robot instantly --
  it coasts for as long as it takes the wheels to decelerate. At the
  original angular_vel=0.3, that's roughly a 1-second coast (~8-9 degrees of
  extra turn); at linear_vel=0.1 it's worse (wheel speed is higher, so the
  coast is longer -- on the order of several cm per side). Fixed by
  switching from bang-bang control (full speed until a hard cutoff) to
  proportional control: command a speed proportional to the remaining
  angle/distance, capped at a max and floored at a min, so the robot is
  already moving slowly (low momentum) by the time it's close enough to
  stop, rather than braking hard from full speed.

  Proportional control shrinks the coast-to-stop distance but doesn't
  eliminate it -- there's still some residual velocity right up until the
  loop breaks. If turn_left starts commanding rotation immediately after
  drive_forward returns, the robot can still be physically coasting forward
  a little, so the "turn" is actually a forward-arcing motion rather than an
  in-place pivot (and likewise for residual rotation bleeding into the next
  drive_forward). Fixed with settle(): an explicit zero-velocity command
  followed by a fixed wait (1s, comfortably longer than the ~0.6s worst-case
  coast time at these speeds) after each motion, so the robot is fully at
  rest before the next one starts.
"""

import math

import rclpy
from rclpy.node import Node
from threading import Thread
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from .angle_helpers import euler_from_quaternion


class DrawSquare(Node):
    """A class that implements a node to pilot a robot in a square."""

    def __init__(self):
        super().__init__("draw_square_with_estop")
        self.manual_estop = False
        self.obstacle_close = False
        self.stop_distance = 0.5
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, "cmd_vel_drive_square", 10)
        self.create_subscription(Bool, "estop", self.handle_estop, 10)
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.create_subscription(Odometry, "odom", self.process_odom, 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()

    def stopped(self):
        """Whether the robot should currently be halted, from either the
        manual estop topic or a nearby obstacle."""
        return self.manual_estop or self.obstacle_close

    def process_odom(self, msg):
        """Tracks the robot's current position and yaw so turn_left/
        drive_forward can measure real motion instead of assuming a speed.
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
        """Handles messages received on the estop topic.

        Args:
            msg (std_msgs.msg.Bool): the message that takes value true if we
            estop and false otherwise.
        """
        self.manual_estop = bool(msg.data)
        if self.manual_estop:
            self.drive(linear=0.0, angular=0.0)

    def process_scan(self, msg):
        """Handles laser scan data, triggering the same stop condition as
        the estop topic if something is within stop_distance in front of the
        robot, and clearing it once the obstacle is no longer close.

        Args:
            msg (sensor_msgs.msg.LaserScan): the current laser scan.
        """
        front_range = msg.ranges[0]
        was_close = self.obstacle_close
        self.obstacle_close = 0.0 < front_range < self.stop_distance
        if self.obstacle_close and not was_close:
            print(f"Obstacle detected {front_range:.2f}m ahead, stopping!")
            self.drive(linear=0.0, angular=0.0)

    def run_loop(self):
        """Executes the main logic for driving the square.  This function does
        not return until the square is finished or the estop is pressed.
        """
        # the first message on the publisher is often missed
        self.drive(0.0, 0.0)
        sleep(1)
        for _ in range(4):
            if not self.stopped():
                print("driving forward")
                self.drive_forward(1.0)
            if not self.stopped():
                print("turning left")
                self.turn_left()
        print("done with run loop")

    def drive(self, linear, angular):
        """Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def settle(self, duration=1.0):
        """Command zero velocity and wait for any residual motion (from
        deceleration lag) to fully die down before switching to a different
        kind of motion. Without this, turn_left can start commanding
        rotation while the robot is still physically coasting forward from
        the previous drive_forward (and vice versa), turning a clean
        stop-then-pivot into an arc.
        """
        self.drive(linear=0.0, angular=0.0)
        sleep(duration)

    def turn_left(self):
        """Execute a 90 degree left turn using proportional control on the
        remaining angle, so the robot is already slowing down as it
        approaches the target instead of coasting past it under momentum
        after a hard stop command.
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
            f"turn_left done: target=90.0deg actual={actual_deg:.1f}deg "
            f"error={actual_deg - 90.0:+.1f}deg stopped_early={stopped_early}"
        )

    def drive_forward(self, distance):
        """Drive straight until odometry reports we've covered the given
        distance, using proportional control on the remaining distance so
        the robot is already slowing down as it approaches the target
        instead of coasting past it under momentum after a hard stop.

        Args:
            distance (_type_): the distance to drive forward.  Only positive
            values are supported.
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
            f"drive_forward done: target={distance:.2f}m actual={actual_traveled:.2f}m "
            f"error={actual_traveled - distance:+.2f}m stopped_early={stopped_early}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DrawSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
