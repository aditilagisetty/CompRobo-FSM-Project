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
  Because e_stop is a threading.Event (thread-safe), the callback thread can
  set it at any time regardless of what run_loop's thread is doing.
- FSM: two states, "driving forward" and "turning left", alternated for 4
  iterations. Transitions are time-based (fixed sleep durations), same as
  Sample 1, just executed sequentially in a dedicated thread instead of a
  timer callback.
- When e-stop triggers, handle_estop/process_scan immediately publish a zero
  Twist and set e_stop, so the robot stops quickly even if run_loop's thread
  is mid-sleep. However, resuming is NOT handled correctly: handle_estop only
  ever calls e_stop.set() when msg.data is True -- there is no `else:
  self.e_stop.clear()` -- so once tripped, e_stop can never be cleared again,
  even if a later message reports the obstacle is gone. The square-drawing
  is stopped for good once e-stop fires.
- It WILL stop after the fourth side (assuming no e-stop): the `for _ in
  range(4)` loop simply exits and the thread ends after 4 forward+turn
  pairs, unlike Sample 1 which loops forever.
"""
import rclpy
from rclpy.node import Node
from threading import Thread, Event
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
        self.e_stop = Event()
        self.stop_distance = 0.5
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()


    def handle_estop(self, msg):
        """Handles messages received on the estop topic.

        Args:
            msg (std_msgs.msg.Bool): the message that takes value true if we
            estop and false otherwise.
        """
        if msg.data:
            self.e_stop.set()
            self.drive(linear=0.0, angular=0.0)

    def process_scan(self, msg):
        """Handles laser scan data, triggering the same e_stop as the estop
        topic if something is within stop_distance in front of the robot.

        Args:
            msg (sensor_msgs.msg.LaserScan): the current laser scan.
        """
        front_range = msg.ranges[0]
        if 0.0 < front_range < self.stop_distance and not self.e_stop.is_set():
            print(f"Obstacle detected {front_range:.2f}m ahead, stopping!")
            self.e_stop.set()
            self.drive(linear=0.0, angular=0.0)

    def run_loop(self):
        """Executes the main logic for driving the square.  This function does
        not return until the square is finished or the estop is pressed.
        """
        # the first message on the publisher is often missed
        self.drive(0.0, 0.0)
        sleep(1)
        for _ in range(4):
            if not self.e_stop.is_set():
                print("driving forward")
                self.drive_forward(0.5)
            if not self.e_stop.is_set():
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
        if not self.e_stop.is_set():
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
        if not self.e_stop.is_set():
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
