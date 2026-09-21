import math
import sys
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from .angle_helpers import euler_from_quaternion
from .icp_matching import (MapPoints, compose, icp_align, inverse, scan_to_points,
                           transform_points, wrap_angle)
from .room_map import DEFAULT_MAP_FILE, PIXEL_FREE, PIXEL_OCCUPIED, SavedMap


class OdomHistory:
    def __init__(self):
        self.samples = deque(maxlen=100)

    def add(self, stamp, pose):
        self.samples.append((stamp, pose))

    def pose_at(self, stamp):
        samples = list(self.samples)
        if len(samples) < 2 or stamp < samples[0][0]:
            return None
        for i in range(len(samples) - 1, 0, -1):
            t0, (x0, y0, a0) = samples[i - 1]
            t1, (x1, y1, a1) = samples[i]
            if t0 <= stamp:
                f = min(1.0, (stamp - t0) / (t1 - t0)) if t1 > t0 else 1.0
                return x0 + f * (x1 - x0), y0 + f * (y1 - y0), wrap_angle(a0 + f * wrap_angle(a1 - a0))


class ICPLocalizer(Node):
    def __init__(self):
        super().__init__('icp_localizer')
        self.declare_parameter('map_file', DEFAULT_MAP_FILE)
        self.declare_parameter('lidar_offset_x', -0.084)
        self.declare_parameter('min_inliers', 0.5)
        self.declare_parameter('max_rms', 0.08)
        self.declare_parameter('max_jump', 0.5)
        self.declare_parameter('max_turn', 0.5)
        self.declare_parameter('gain', 0.5)
        self.declare_parameter('max_spin', 0.4)
        self.map_file = self.get_parameter('map_file').value
        self.lidar_offset_x = self.get_parameter('lidar_offset_x').value
        self.min_inliers = self.get_parameter('min_inliers').value
        self.max_rms = self.get_parameter('max_rms').value
        self.max_jump = self.get_parameter('max_jump').value
        self.max_turn = self.get_parameter('max_turn').value
        self.gain = self.get_parameter('gain').value
        self.max_spin = self.get_parameter('max_spin').value

        self.saved_map = SavedMap.load(self.map_file)
        self.map_points = MapPoints(self.saved_map)
        self.correction = (0.0, 0.0, 0.0)
        self.odom_pose = (0.0, 0.0, 0.0)
        self.odom_stamp = None
        self.have_odom = False
        self.spin_rate = 0.0
        self.history = OdomHistory()
        self.icp_ready = True
        self.scans_matched = 0
        self.scans_rejected = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseStamped, 'localized_pose', 10)
        self.scan_pub = self.create_publisher(Marker, 'scan_in_map', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'saved_map', latched)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_timer(2.0, self.report_status)
        self.publish_saved_map()

    def process_odom(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.odom_stamp = msg.header.stamp
        self.spin_rate = abs(msg.twist.twist.angular.z)
        self.history.add(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, self.odom_pose)
        self.have_odom = True
        self.publish_estimate()

    def process_scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odom_at_scan = self.history.pose_at(stamp)
        if odom_at_scan is None or self.spin_rate > self.max_spin:
            return
        points = scan_to_points(msg.ranges, msg.angle_increment, msg.range_min,
                                msg.range_max, self.lidar_offset_x)
        if len(points) < 20:
            return
        predicted = compose(self.correction, odom_at_scan)
        try:
            result = icp_align(points, self.map_points, predicted)
        except NotImplementedError:
            if self.icp_ready:
                self.get_logger().warn('icp_align is not implemented yet, passing odometry through')
                self.icp_ready = False
            result = None
        estimate = self.fuse(predicted, result)
        self.correction = compose(estimate, inverse(odom_at_scan))
        self.publish_scan_in_map(points, estimate)

    def fuse(self, predicted, result):
        if result is None:
            return predicted
        pose, inlier_fraction, rms = result
        jump = math.hypot(pose[0] - predicted[0], pose[1] - predicted[1])
        turn = abs(wrap_angle(pose[2] - predicted[2]))
        if (inlier_fraction < self.min_inliers or rms > self.max_rms
                or jump > self.max_jump or turn > self.max_turn):
            self.scans_rejected += 1
            return predicted
        self.scans_matched += 1
        g = self.gain
        return (predicted[0] + g * (pose[0] - predicted[0]),
                predicted[1] + g * (pose[1] - predicted[1]),
                wrap_angle(predicted[2] + g * wrap_angle(pose[2] - predicted[2])))

    def publish_estimate(self):
        x, y, yaw = compose(self.correction, self.odom_pose)

        tf = TransformStamped()
        tf.header.stamp = self.odom_stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = self.correction[0]
        tf.transform.translation.y = self.correction[1]
        tf.transform.rotation.z = math.sin(self.correction[2] / 2)
        tf.transform.rotation.w = math.cos(self.correction[2] / 2)
        self.tf_broadcaster.sendTransform(tf)

        pose = PoseStamped()
        pose.header.stamp = self.odom_stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        self.pose_pub.publish(pose)

    def publish_scan_in_map(self, points, robot_pose):
        marker = Marker()
        marker.header.stamp = self.odom_stamp
        marker.header.frame_id = 'map'
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.points = [Point(x=float(px), y=float(py), z=0.0)
                         for px, py in transform_points(points, robot_pose)]
        self.scan_pub.publish(marker)

    def publish_saved_map(self):
        grid = np.full(self.saved_map.image.shape, -1, dtype=np.int8)
        grid[self.saved_map.image == PIXEL_FREE] = 0
        grid[self.saved_map.image == PIXEL_OCCUPIED] = 100
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.saved_map.resolution
        msg.info.width = self.saved_map.width
        msg.info.height = self.saved_map.height
        msg.info.origin.position.x = self.saved_map.origin_x
        msg.info.origin.position.y = self.saved_map.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = np.flipud(grid).reshape(-1).tolist()
        self.map_pub.publish(msg)

    def report_status(self):
        if not self.have_odom:
            return
        x, y, yaw = compose(self.correction, self.odom_pose)
        self.get_logger().info(
            f'odom=({self.odom_pose[0]:.2f}, {self.odom_pose[1]:.2f}) '
            f'localized=({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}deg) '
            f'correction=({self.correction[0]:.2f}, {self.correction[1]:.2f}, '
            f'{math.degrees(self.correction[2]):.0f}deg) '
            f'matched={self.scans_matched} rejected={self.scans_rejected}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ICPLocalizer()
    except OSError:
        print('No map found. Run teleop_scan first and save a map, or pass -p map_file:=...')
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
