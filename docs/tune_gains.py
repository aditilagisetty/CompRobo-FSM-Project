"""Offline sweep behind the k_repulsive/k_steer choice in
collision_avoidance.py

Run with ROS sourced:
    source /opt/ros/jazzy/setup.bash
    PYTHONNOUSERSITE=1 python3 docs/tune_gains.py
"""
import math
import os
import sys

import rclpy
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from ros_behaviors_fsm.collision_avoidance import CollisionAvoidance


def make_scan(angle_increment_deg=1.0, populate=None):
    n = round(360.0 / angle_increment_deg)
    msg = LaserScan()
    msg.angle_increment = math.radians(angle_increment_deg)
    msg.ranges = [float('inf')] * n
    for deg, r in (populate or {}).items():
        msg.ranges[int(round(deg / angle_increment_deg)) % n] = r
    return msg


def wall_rays(distance, center_deg=90, span_deg=90, step=5):
    """Many rays populated on one side, like actually driving parallel to a
    wall -- not just one ray, which understates a real wall's push.
    """
    return {d: distance for d in range(center_deg - span_deg // 2, center_deg + span_deg // 2 + 1, step)}


def eval_case(node, populate):
    net_x, net_y = node.compute_potential_field(make_scan(populate=populate))
    heading = math.atan2(net_y, net_x)
    angular = max(-node.max_angular_speed, min(node.max_angular_speed, node.k_steer * heading))
    repulsion_magnitude = math.hypot(net_x - node.k_attractive, net_y)
    linear = node.forward_speed / (1.0 + repulsion_magnitude) if net_x > 0 else 0.0
    return linear, angular


def main():
    rclpy.init()

    print("gain sweep:")
    for k_rep in [0.02, 0.04, 0.06, 0.08, 0.10]:
        for k_steer in [1.0, 1.5, 2.0]:
            node = CollisionAvoidance()
            node.k_repulsive, node.k_steer = k_rep, k_steer
            open_ = eval_case(node, {})
            wall05 = eval_case(node, wall_rays(0.5))
            wall07 = eval_case(node, wall_rays(0.7))
            single = eval_case(node, {90: 0.5})
            print(f"  k_rep={k_rep:.2f} k_steer={k_steer:.1f} | "
                  f"open=({open_[0]:.3f},{open_[1]:.3f}) "
                  f"wall@0.5m={wall05[0]:.3f},{wall05[1]:.3f} "
                  f"wall@0.7m={wall07[0]:.3f},{wall07[1]:.3f} "
                  f"single-ray@0.5m={single[0]:.3f},{single[1]:.3f}")
            node.destroy_node()

    print("\nboundary check just past the hard-stop release distance (0.42m), k_rep=0.02:")
    for k_steer in [1.0, 1.5, 2.0]:
        node = CollisionAvoidance()
        node.k_repulsive, node.k_steer = 0.02, k_steer
        lin, ang = eval_case(node, wall_rays(0.42))
        print(f"  k_steer={k_steer:.1f}: linear={lin:.3f} angular={ang:.3f}")
        node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
