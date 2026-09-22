"""Plots the potential-field bug and fix described in WRITEUP.md's collision
avoidance section: an obstacle directly ahead used to produce a net force
sitting exactly on the atan2 +/-pi discontinuity, and the fix (a fixed
rotational bias on every repulsive vector) moves it off that point.

Uses the real compute_potential_field from collision_avoidance.py, not
illustrative numbers. Run with ROS sourced:

    source /opt/ros/jazzy/setup.bash
    PYTHONNOUSERSITE=1 python3 docs/plot_potential_field_fix.py
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from ros_behaviors_fsm.collision_avoidance import CollisionAvoidance

RED, GREEN, BLUE, GREY, AMBER = '#DC2626', '#16A34A', '#2563EB', '#9CA3AF', '#D97706'

plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 10.5, 'axes.titlesize': 12, 'axes.titleweight': 'bold',
    'figure.facecolor': 'white', 'axes.edgecolor': '#9CA3AF',
})


class FakeScan:
    def __init__(self, ranges, inc):
        self.ranges = ranges
        self.angle_increment = inc


def symmetric_scan(distance=0.4, half_width_deg=30, n=360):
    ranges = [float('inf')] * n
    inc = 2 * math.pi / n
    for deg in range(-half_width_deg, half_width_deg + 1):
        idx = round(math.radians(deg) / inc) % n
        ranges[idx] = distance
    return FakeScan(ranges, inc)


def obstacle_points(scan, node):
    pts = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r <= 0.0 or r >= node.influence_radius:
            continue
        theta = i * scan.angle_increment
        pts.append((r * math.cos(theta), r * math.sin(theta)))
    return np.array(pts)


def draw_panel(ax, node, scan, bias, title):
    node.repulsion_bias_angle = bias
    net_x, net_y = node.compute_potential_field(scan)
    heading = math.atan2(net_y, net_x)

    pts = obstacle_points(scan, node)
    ax.plot(pts[:, 0], pts[:, 1], '.', color=RED, ms=4, label='obstacle (scan points)', zorder=3)
    ax.plot(0, 0, 'o', color='#1F2937', ms=10, zorder=5)
    ax.annotate('', xy=(0.35, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle='-|>', color=GREY, lw=1.5))
    ax.text(0.37, 0.02, 'forward\n(robot frame)', color=GREY, fontsize=8.5, va='bottom')

    # a few individual repulsion vectors, lightly, for intuition
    rng = np.random.default_rng(0)
    sample = pts[rng.choice(len(pts), size=min(10, len(pts)), replace=False)]
    for px, py in sample:
        r = math.hypot(px, py)
        theta = math.atan2(py, px)
        mag = node.k_repulsive * (1.0 / r - 1.0 / node.influence_radius)
        ang = theta + math.pi + bias
        dx, dy = 0.5 * mag / node.k_repulsive * math.cos(ang), 0.5 * mag / node.k_repulsive * math.sin(ang)
        ax.annotate('', xy=(px + dx, py + dy), xytext=(px, py),
                    arrowprops=dict(arrowstyle='-|>', color=GREY, lw=0.8, alpha=0.6))

    scale = 0.5
    ax.annotate('', xy=(scale * net_x, scale * net_y), xytext=(0, 0),
                arrowprops=dict(arrowstyle='-|>', color=BLUE, lw=3))
    ax.text(0.03, 0.03, f'net force heading = {math.degrees(heading):+.1f}deg',
            color=BLUE, fontsize=10, fontweight='bold', ha='left', va='top',
            transform=ax.transAxes, bbox=dict(facecolor='white', edgecolor=BLUE, boxstyle='round,pad=0.35'))

    ax.set_xlim(-0.7, 0.7)
    ax.set_ylim(-0.6, 0.6)
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.set_xlabel('x (m)')
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.grid(True, color='#F3F4F6')
    ax.axhline(0, color='#E5E7EB', lw=0.8, zorder=0)
    ax.axvline(0, color='#E5E7EB', lw=0.8, zorder=0)


def main():
    rclpy.init()
    node = CollisionAvoidance()
    scan = symmetric_scan()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5.2))
    draw_panel(ax1, node, scan, 0.0, 'Before the fix\n(no rotational bias)')
    ax1.set_ylabel('y (m)')
    draw_panel(ax2, node, scan, math.radians(20), 'After the fix\n(20deg rotational bias)')

    fig.suptitle('An obstacle straight ahead: why the net force used to point\n'
                 'almost exactly backward, and how the fix moves it off that point',
                 fontsize=12.5, fontweight='bold', y=1.03)
    fig.text(0.5, -0.03,
             'Left: repulsion from a symmetric obstacle sums to a vector at 180.0deg -- exactly on the atan2 +/-180deg\n'
             'discontinuity, where floating-point noise flips the sign each scan. Right: rotating every repulsion\n'
             'vector by a fixed 20deg moves the net force well off that point, so it turns the same way every time.',
             ha='center', fontsize=9.5, color='#4B5563')

    out = os.path.join(os.path.dirname(__file__), 'figures', 'potential_field_fix.png')
    fig.savefig(out, dpi=200, bbox_inches='tight')
    print('wrote', out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
