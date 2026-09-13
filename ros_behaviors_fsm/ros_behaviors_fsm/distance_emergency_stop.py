import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class DistanceEmergencyStopNode(Node):
    def __init__(self):
        super().__init__('distance_emergency_stop_node')
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.stop_distance = 0.5
        self.too_close = False
        self.timer = self.create_timer(0.1, self.run_loop)

    def process_scan(self, msg):
        front_range = msg.ranges[0]
        # 0.0 means "no return" on this sim's lidar, not "touching it"
        self.too_close = 0.0 < front_range < self.stop_distance

    def run_loop(self):
        vel = Twist()
        if not self.too_close:
            vel.linear.x = 0.1
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceEmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
