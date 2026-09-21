import math

import numpy as np

from .room_map import PIXEL_OCCUPIED


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return ax + c * bx - s * by, ay + s * bx + c * by, wrap_angle(at + bt)


def inverse(pose):
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    return -(c * x + s * y), s * x - c * y, wrap_angle(-t)


def transform_points(points, pose):
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    rotation = np.array([[c, -s], [s, c]])
    return points @ rotation.T + np.array([x, y])


def scan_to_points(ranges, angle_increment, range_min, range_max, lidar_offset_x=0.0):
    ranges = np.asarray(ranges, dtype=float)
    angles = np.arange(len(ranges)) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges < range_max)
    r, a = ranges[valid], angles[valid]
    return np.column_stack([r * np.cos(a) + lidar_offset_x, r * np.sin(a)])


class MapPoints:
    def __init__(self, saved_map):
        rows, cols = np.nonzero(saved_map.image == PIXEL_OCCUPIED)
        xs, ys = saved_map.pixel_to_world(cols + 0.5, rows + 0.5)
        self.points = np.column_stack([xs, ys])

    def __len__(self):
        return len(self.points)

    def nearest(self, points):
        d2 = ((points ** 2).sum(1)[:, None] + (self.points ** 2).sum(1)[None, :]
              - 2.0 * points @ self.points.T)
        index = d2.argmin(axis=1)
        distance = np.sqrt(np.maximum(d2[np.arange(len(points)), index], 0.0))
        return self.points[index], distance


def find_correspondences(scan_points, map_points, max_distance):
    """scan_points: (N, 2) already in the map frame.
    Pair every scan point with its nearest occupied map point
    (map_points.nearest) and drop pairs you don't trust.
    Return (scan_pts, map_pts), both (K, 2), row i of one matching row i of the other.
    """
    nearest_points, distance = map_points.nearest(scan_points)
    trusted = distance < max_distance
    return scan_points[trusted], nearest_points[trusted]


def best_rigid_transform(source, target):
    """(K, 2) point sets with row i of source matching row i of target.
    Return (dx, dy, dyaw) minimizing sum |R(dyaw) @ s_i + (dx, dy) - t_i|^2.
    """
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    s = source - source_center
    t = target - target_center
    dot = (s[:, 0] * t[:, 0] + s[:, 1] * t[:, 1]).sum()
    cross = (s[:, 0] * t[:, 1] - s[:, 1] * t[:, 0]).sum()
    dyaw = math.atan2(cross, dot)
    c, sn = math.cos(dyaw), math.sin(dyaw)
    dx = target_center[0] - (c * source_center[0] - sn * source_center[1])
    dy = target_center[1] - (sn * source_center[0] + c * source_center[1])
    return float(dx), float(dy), dyaw


def icp_align(scan_points, map_points, guess, max_iterations=20, max_distance=0.5):
    """scan_points: (N, 2) in the robot frame. guess: (x, y, yaw) of the robot in the map.
    Return (pose, inlier_fraction, rms_error): the refined (x, y, yaw), the fraction
    of scan points that ended up with a trusted match, and the rms distance of those
    matches in meters. Return None if it cannot produce an estimate.
    """
    if len(scan_points) < 3 or len(map_points) == 0:
        return None
    pose = tuple(guess)
    for _ in range(max_iterations):
        in_map = transform_points(scan_points, pose)
        source, target = find_correspondences(in_map, map_points, max_distance)
        if len(source) < 3:
            return None
        step = best_rigid_transform(source, target)
        pose = compose(step, pose)
        if abs(step[0]) < 1e-4 and abs(step[1]) < 1e-4 and abs(step[2]) < 1e-4:
            break
    in_map = transform_points(scan_points, pose)
    source, target = find_correspondences(in_map, map_points, max_distance)
    if len(source) < 3:
        return None
    inlier_fraction = len(source) / len(scan_points)
    rms = float(np.sqrt(((source - target) ** 2).sum(axis=1).mean()))
    return pose, inlier_fraction, rms
