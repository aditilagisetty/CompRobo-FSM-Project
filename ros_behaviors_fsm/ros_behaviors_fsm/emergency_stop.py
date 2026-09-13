import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self.sub = self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bumped = False
        self.timer = self.create_timer(0.1, self.run_loop)

    def process_bump(self, msg):
        if msg.left_front or msg.left_side or msg.right_front or msg.right_side:
            self.bumped = True
    
    def run_loop(self):
        vel = Twist()
        if not self.bumped:
            vel.linear.x = 0.1  
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()