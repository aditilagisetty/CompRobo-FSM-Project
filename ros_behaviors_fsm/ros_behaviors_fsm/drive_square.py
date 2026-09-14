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
  One caveat that's a known simplification of this time-based approach: the
  stop/resume check only happens between segments (drive_forward/turn_left),
  not continuously during a segment's sleep -- so an interruption mid-segment
  can still cause that segment's move to be skipped rather than resumed
  exactly where it left off.
"""
import rclpy
from rclpy.node import Node
from threading import Thread
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class DrawSquare(Node):
    """A class that implements a node to pilot a robot in a square.
    """

    def __init__(self):
        super().__init__('draw_square_with_estop')
        self.manual_estop = False
        self.obstacle_close = False
        self.stop_distance = 0.5
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()

    def stopped(self):
        """Whether the robot should currently be halted, from either the
        manual estop topic or a nearby obstacle."""
        return self.manual_estop or self.obstacle_close

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
        print('done with run loop')

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

    def turn_left(self):
        """Execute a 90 degree left turn
        """
        angular_vel = 0.3
        if not self.stopped():
            self.drive(linear=0.0, angular=angular_vel)
            sleep(math.pi / angular_vel / 2)
            self.drive(linear=0.0, angular=0.0)

    def drive_forward(self, distance):
        """Drive straight for the spefcified distance.

        Args:
            distance (_type_): the distance to drive forward.  Only positive
            values are supported.
        """
        forward_vel = 0.1
        if not self.stopped():
            self.drive(linear=forward_vel, angular=0.0)
        sleep(distance / forward_vel)
        self.drive(linear=0.0, angular=0.0)

def main(args=None):
    rclpy.init(args=args)
    node = DrawSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
