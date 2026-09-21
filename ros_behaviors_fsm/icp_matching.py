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
    raise NotImplementedError


def best_rigid_transform(source, target):
    """(K, 2) point sets with row i of source matching row i of target.
    Return (dx, dy, dyaw) minimizing sum |R(dyaw) @ s_i + (dx, dy) - t_i|^2.
    """
    raise NotImplementedError


def icp_align(scan_points, map_points, guess, max_iterations=20, max_distance=0.5):
    """scan_points: (N, 2) in the robot frame. guess: (x, y, yaw) of the robot in the map.
    Return (pose, inlier_fraction, rms_error): the refined (x, y, yaw), the fraction
    of scan points that ended up with a trusted match, and the rms distance of those
    matches in meters. Return None if it cannot produce an estimate.
    """
    raise NotImplementedError
