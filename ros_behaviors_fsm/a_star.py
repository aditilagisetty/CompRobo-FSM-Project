"""
Searches SavedMap's pixel grid built by teleop_scan.py for a path from a
start to a goal, avoiding occupied cells nflated by the robot's radius, so
the found path keeps the robot's whole body clear of walls, not just its
center point. plan_world_path() is the convenience entry point
path_following.py calls and gives it (x, y) meters and get back
a list of (x, y) waypoints, which get fed through the same
resample_path/smooth_path used for hand-drawn paths, then
into PathFollower.follow(waypoints)
"""
import heapq
import math

import numpy as np

from .room_map import PIXEL_OCCUPIED

DEFAULT_ROBOT_RADIUS = 0.16  # meters


def inflate_obstacles(image, radius_cells):
    """Returns a boolean array, True wherever a cell is occupied or within
    radius_cells of an occupied cell -- i.e. where the robot's center could
    not be without some part of its body overlapping an obstacle.
    """
    occupied = image == PIXEL_OCCUPIED
    inflated = occupied.copy()
    height, width = occupied.shape
    for d_row in range(-radius_cells, radius_cells + 1):
        for d_col in range(-radius_cells, radius_cells + 1):
            if d_row == 0 and d_col == 0:
                continue
            if d_row * d_row + d_col * d_col > radius_cells * radius_cells:
                continue  # inflated shape kinda circular
            shifted = np.zeros_like(occupied)
            src_r0, src_r1 = max(0, -d_row), height - max(0, d_row)
            src_c0, src_c1 = max(0, -d_col), width - max(0, d_col)
            dst_r0, dst_r1 = max(0, d_row), height - max(0, -d_row)
            dst_c0, dst_c1 = max(0, d_col), width - max(0, -d_col)
            shifted[dst_r0:dst_r1, dst_c0:dst_c1] = occupied[src_r0:src_r1, src_c0:src_c1]
            inflated |= shifted
    return inflated


def neighbors(inflated, cell):
    """gets (neighbor_cell, step_cost) for every valid 8-directional
    neighbor of `cell` that's in bounds and not occupied
    """
    height, width = inflated.shape
    row, col = cell
    for d_row in (-1, 0, 1):
        for d_col in (-1, 0, 1):
            if d_row == 0 and d_col == 0:
                continue
            n_row, n_col = row + d_row, col + d_col
            if not (0 <= n_row < height and 0 <= n_col < width):
                continue
            if inflated[n_row, n_col]:
                continue
            step_cost = math.hypot(d_row, d_col)
            yield (n_row, n_col), step_cost


def straight_line_distance(a, b):
    """straight line distance between two (row, col) cells"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def find_path(inflated, start, goal):
    """A* search from start to goal over a precomputed boolean array

    Returns a list of (row, col) cells from start to goal or
    None if no path exists including if start or goal is itself blocked
    """
    if inflated[start] or inflated[goal]:
        return None

    open_heap = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    visited = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return _reconstruct_path(came_from, current)

        for neighbor, step_cost in neighbors(inflated, current):
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + straight_line_distance(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return None


def _reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def plan_world_path(saved_map, start_xy, goal_xy, robot_radius=DEFAULT_ROBOT_RADIUS):
    """takes start/goal as (x, y) world coordinates,
    inflates the map's obstacles by robot_radius, runs A* over the
    resulting grid, and returns the path as a list of (x, y) world
    coordinates, or None if no path was found
    """
    radius_cells = max(1, int(round(robot_radius / saved_map.resolution)))
    inflated = inflate_obstacles(saved_map.image, radius_cells)

    start_col, start_row = saved_map.world_to_pixel(*start_xy)
    goal_col, goal_row = saved_map.world_to_pixel(*goal_xy)
    start_cell = (int(round(start_row)), int(round(start_col)))
    goal_cell = (int(round(goal_row)), int(round(goal_col)))

    cell_path = find_path(inflated, start_cell, goal_cell)
    if cell_path is None:
        return None

    return [saved_map.pixel_to_world(col, row) for row, col in cell_path]
