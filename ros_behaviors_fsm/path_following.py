"""
Draw a route on the saved map, or right-click a goal for A* to plan one,
and follow it with pure pursuit. Also hosts the always-visible FSM control
panel (mode buttons, live status, arrow-key driving) this app opens with.
"""

import math
import queue
import sys
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from threading import Lock, Thread
from queue import Queue

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from neato2_interfaces.msg import Bump
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String

from .a_star import plan_world_path
from .angle_helpers import euler_from_quaternion
from .room_map import DEFAULT_MAP_FILE, SavedMap

TURN_IN_PLACE_ANGLE = math.radians(50)
FRONT_CONE = math.radians(20)


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def resample_path(points, spacing):
    """
    Return points spaced `spacing` apart along the polyline through
    points, first and last kept, so a hand-drawn stroke's uneven sampling
    doesn't affect the follower.
    """
    if len(points) < 2:
        return list(points)
    pts = np.asarray(points, dtype=float)
    seg = np.hypot(*np.diff(pts, axis=0).T)
    along = np.concatenate([[0.0], np.cumsum(seg)])
    total = along[-1]
    if total < 1e-9:
        return [tuple(pts[0])]
    targets = np.append(np.arange(0.0, total, spacing), total)
    xs = np.interp(targets, along, pts[:, 0])
    ys = np.interp(targets, along, pts[:, 1])
    return list(zip(xs.tolist(), ys.tolist()))


def smooth_path(points, passes=2):
    """
    Average each interior point with its neighbors, a few passes, to
    take the wobble out of a hand-drawn stroke; endpoints are left alone.
    """
    pts = np.asarray(points, dtype=float)
    for _ in range(passes):
        if len(pts) < 3:
            break
        pts[1:-1] = (pts[:-2] + pts[1:-1] + pts[2:]) / 3.0
    return [tuple(p) for p in pts.tolist()]


class PathFollower(Node):
    """
    Follows a waypoint list with pure pursuit, pausing for a bump, an
    e-stop, or anything close ahead, and reports its status and pose for
    the drawing window and the gateway FSM to watch.
    """

    def __init__(self, ui_queue: Queue):
        """
        Declare the follower's tuning parameters and wire up its
        cmd_vel/status/path publishers and its odom/scan/bump/estop/
        current_mode subscriptions.
        """
        super().__init__("path_following")
        self.declare_parameter("map_file", DEFAULT_MAP_FILE)
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("k_heading", 1.5)
        self.declare_parameter("lookahead", 0.3)
        self.declare_parameter("goal_tolerance", 0.08)
        self.declare_parameter("stop_distance", 0.25)
        self.declare_parameter("waypoint_spacing", 0.15)
        self.declare_parameter("robot_radius", 0.28)
        self.map_file = self.get_parameter("map_file").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.k_heading = self.get_parameter("k_heading").value
        self.lookahead = self.get_parameter("lookahead").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.stop_distance = self.get_parameter("stop_distance").value
        self.waypoint_spacing = self.get_parameter("waypoint_spacing").value
        self.robot_radius = self.get_parameter("robot_radius").value
        self.ui_queue = ui_queue

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.manual_estop = False
        self.bumped = False
        self.bump_timeout_sec = 0.3
        self.last_bump_time = None
        self.obstacle_close = False

        self.lock = Lock()
        self.path = []
        self.idx = 0
        self.state = "idle"

        self.current_mode = "WALL FOLLOW"  # fsm_node's own startup default
        self.mode_history = deque(maxlen=200)

        self.create_subscription(String, "current_mode", self.mode_callback, 10)
        # lets the control panel switch fsm_node's mode without a separate
        # `ros2 topic pub` -- a long-lived publisher in the same process
        # avoids the discovery-race that can drop a short-lived `pub -1`
        self.fsm_command_pub = self.create_publisher(String, "fsm_command", 10)
        # lets the control panel's arrow keys drive teleop_scan.py -- Tk
        # reads keys from the display's keyboard focus, not this process's
        # stdin, so this works even when nothing here has a real terminal
        self.teleop_vel_pub = self.create_publisher(Twist, "cmd_vel_teleop_scan", 10)
        self.save_map_pub = self.create_publisher(Empty, "save_map_command", 10)

        self.vel_pub = self.create_publisher(Twist, "cmd_vel_path_following", 10)
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "drawn_path", path_qos)
        # Reports whether this node currently owns cmd_vel (active or
        # paused-but-not-done, vs idle/done) -- e.g. so an FSM watching this
        # topic knows when to step back and when to resume its own driving.
        # fsm_node.py (this project's FSM) doesn't subscribe to it; it uses
        # current_mode instead (above) to know when to show its own GUI.
        self.status_pub = self.create_publisher(String, "path_following_status", 10)
        self.create_subscription(Odometry, "odom", self.process_odom, 10)
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.create_subscription(Bump, "bump", self.process_bump, 10)
        self.create_subscription(Bool, "estop", self.handle_estop, 10)
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.1, self.publish_status)

    def mode_callback(self, msg):
        """
        Track the gateway FSM's current mode, log it, and tell the
        control panel to show or hide the drawing window accordingly.
        """
        self.current_mode = msg.data
        self.mode_history.append((datetime.now().strftime("%H:%M:%S"), msg.data))
        self.ui_queue.put("SHOW_UI" if msg.data == "PATH FOLLOWING" else "HIDE_UI")

    def send_fsm_command(self, key):
        """
        Same t/m/g/p switches fsm_node's own keyboard listener sends,
        published from this long-lived node instead of a one-shot CLI pub.
        """
        self.fsm_command_pub.publish(String(data=key))

    def drive_teleop(self, linear, angular):
        """
        Drives teleop_scan.py's robot -- only takes effect while fsm_node
        is in TELEOP SCAN mode, same as pressing wasd in its own terminal.
        """
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.teleop_vel_pub.publish(msg)

    def save_teleop_map(self):
        """
        Tells teleop_scan.py to save its map now (same as pressing m in
        its own terminal); works even if that process has no real terminal.
        """
        self.save_map_pub.publish(Empty())

    def process_odom(self, msg):
        """Track the robot's current pose from odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def process_scan(self, msg):
        """
        Flag whether anything is within stop_distance in the front
        cone, for control_loop to pause on.
        """
        ranges = np.asarray(msg.ranges, dtype=float)
        # ranges[0] is the front and beams go counterclockwise
        angles = np.arange(len(ranges)) * msg.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        in_cone = np.abs(angles) < FRONT_CONE
        r = ranges[in_cone]
        r = r[np.isfinite(r) & (r > msg.range_min)]
        self.obstacle_close = bool(len(r) and r.min() < self.stop_distance)

    def process_bump(self, msg):
        """
        Set self.bumped True on any real bump message; check_bump_timeout
        is what clears it, since the simulator never sends an explicit
        "bump cleared" message.
        """
        if msg.left_front or msg.left_side or msg.right_front or msg.right_side:
            self.last_bump_time = time.monotonic()
            self.bumped = True

    def check_bump_timeout(self):
        """
        Clear self.bumped once bump_timeout_sec has passed with no new
        bump message.
        """
        if (
            self.bumped
            and time.monotonic() - self.last_bump_time > self.bump_timeout_sec
        ):
            self.bumped = False

    def handle_estop(self, msg):
        """Track the manual e-stop topic's current value."""
        self.manual_estop = bool(msg.data)

    def follow(self, waypoints):
        """
        Start following the given (x, y) waypoint list from its start,
        and publish it for RViz.
        """
        with self.lock:
            self.path = list(waypoints)
            self.idx = 0
            self.state = "following"
        self.publish_path(waypoints)
        self.get_logger().info(f"following a path of {len(waypoints)} waypoints")

    def cancel(self):
        """Stop following, clear the path, and command zero velocity."""
        with self.lock:
            self.state = "idle"
            self.path = []
        self.drive(0.0, 0.0)

    def publish_path(self, waypoints):
        """Publish the waypoint list as a Path message, for RViz."""
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def drive(self, linear, angular):
        """Publish a Twist with the given linear and angular velocity."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)

    def status_text(self):
        """
        A short human-readable summary of the current following state,
        for the drawing window's status line.
        """
        with self.lock:
            state, idx, total = self.state, self.idx, len(self.path)
        if state == "following":
            return f"following: waypoint {idx + 1}/{total}"
        if state == "paused":
            reason = (
                "e-stop"
                if self.manual_estop
                else "bumped" if self.bumped else "obstacle ahead"
            )
            return f"paused ({reason}): waypoint {idx + 1}/{total}"
        return state

    def publish_status(self):
        """
        Publish the current following state (idle/following/paused/done)
        on path_following_status.
        """
        with self.lock:
            state = self.state
        self.status_pub.publish(String(data=state))

    def control_loop(self):
        """
        Pause for a bump, e-stop, or close obstacle; resume and steer
        otherwise; mark done once the path is finished.
        """
        self.check_bump_timeout()
        with self.lock:
            if self.state not in ("following", "paused") or not self.have_odom:
                return
            if self.manual_estop or self.bumped or self.obstacle_close:
                if self.state == "following":
                    self.get_logger().info("paused: something is in the way")
                self.state = "paused"
                self.drive(0.0, 0.0)
                return
            if self.state == "paused":
                self.get_logger().info("clear again, resuming")
                self.state = "following"
            linear, angular, finished = self.steer()
            if finished:
                self.state = "done"
                self.path = []
                self.get_logger().info("reached the end of the path")
        self.drive(linear, angular)

    def steer(self):
        """
        Pure pursuit: advance to the nearest upcoming waypoint, aim at
        the point lookahead meters further along, turn in place for a large
        heading error, and slow down near the goal.
        Return (linear, angular, finished).
        """
        path = self.path

        def dist(i):
            """Distance from the robot's current position to path[i]."""
            return math.hypot(path[i][0] - self.x, path[i][1] - self.y)

        last = len(path) - 1
        window = range(self.idx, min(last, self.idx + 6) + 1)
        self.idx = min(window, key=dist)
        goal_dist = dist(last)
        if self.idx == last and goal_dist < self.goal_tolerance:
            return 0.0, 0.0, True

        target = self.idx
        along = dist(self.idx)
        while target < last and along < self.lookahead:
            along += math.dist(path[target], path[target + 1])
            target += 1
        heading = math.atan2(path[target][1] - self.y, path[target][0] - self.x)
        error = wrap_angle(heading - self.yaw)
        angular = max(
            -self.max_angular_speed, min(self.max_angular_speed, self.k_heading * error)
        )
        if abs(error) > TURN_IN_PLACE_ANGLE:
            return 0.0, angular, False
        linear = self.linear_speed * (1.0 - abs(error) / TURN_IN_PLACE_ANGLE)
        if target == last:
            linear = min(linear, max(0.03, 0.5 * goal_dist))
        return linear, angular, False


class PathPainter:
    """
    

    GUI for drawing a path on a map and sending it to the PathFollower node.
    The user can draw strokes on the map, set a goal point, and plan a path using A* algorithm.
    The GUI also allows the user to clear the strokes, undo the last stroke, and reload the map.
    """

    MAX_WIDTH = 1000
    MAX_HEIGHT = 700
    REFRESH_MS = 100

    def __init__(self, node, master):
        """
        Build the drawing window (as a Toplevel under master) and its
        buttons, load the current map, and start out hidden.
        """
        self.node = node
        self.map = None
        self.strokes = []
        self.waypoints = []
        self.bad_points = []
        self.goal_click = None

        # a Toplevel under the control panel's persistent root, not its own
        # Tk() -- so it can be shown/hidden as fsm_node's mode changes
        # instead of being fully created and destroyed each time, which let
        # this window coexist with the always-visible control panel
        self.root = tk.Toplevel(master)
        self.root.title("Neato path painter")
        bar = tk.Frame(self.root)
        bar.pack(side=tk.TOP, fill=tk.X)
        for label, command in (
            ("Go", self.go),
            ("Stop", self.stop),
            ("Undo", self.undo),
            ("Clear", self.clear),
            ("Reload map", self.reload_map),
            ("Plan (A*)", self.plan_astar),
        ):
            tk.Button(bar, text=label, command=command).pack(
                side=tk.LEFT, padx=2, pady=2
            )
        self.status = tk.Label(bar, text="", anchor="w")
        self.status.pack(side=tk.LEFT, padx=10)
        self.canvas = tk.Canvas(self.root, highlightthickness=0, cursor="pencil")
        self.canvas.pack()
        self.message = tk.Label(self.root, text="", anchor="w")
        self.message.pack(side=tk.BOTTOM, fill=tk.X)

        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonPress-3>", self.on_click_goal)
        self.root.bind("<Return>", lambda e: self.go())
        self.root.bind("<space>", lambda e: self.stop())
        self.root.bind("<Control-z>", lambda e: self.undo())
        self.root.bind("c", lambda e: self.clear())
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.reload_map()
        self.root.after(self.REFRESH_MS, self.tick)
        self.root.withdraw()  # hidden until fsm_node actually enters PATH FOLLOWING

    def reload_map(self):
        """
        Load (or reload) the map file from disk and redraw the canvas,
        clearing any in-progress drawing.
        """
        try:
            self.map = SavedMap.load(self.node.map_file)
        except (OSError, KeyError, ValueError) as err:
            self.set_message(f"could not load {self.node.map_file}: {err}")
            return
        c0, r0, c1, r1 = self.map.explored_bounds()
        self.crop = (c0, r0)
        width, height = c1 - c0, r1 - r0
        self.zoom = max(1, min(self.MAX_WIDTH // width, self.MAX_HEIGHT // height))
        pgm = b"P5\n%d %d\n255\n" % (width, height)
        pgm += np.ascontiguousarray(self.map.image[r0:r1, c0:c1]).tobytes()
        self.photo = tk.PhotoImage(data=pgm).zoom(self.zoom)
        self.canvas.config(width=width * self.zoom, height=height * self.zoom)
        self.strokes = []
        self.waypoints = []
        self.bad_points = []
        self.goal_click = None
        self.redraw()
        self.set_message(
            f"map: {self.node.map_file}  "
            f"({width * self.map.resolution:.1f} x "
            f"{height * self.map.resolution:.1f} m shown). "
            "Draw a path, then press Go."
        )

    def canvas_to_world(self, cx, cy):
        """Convert a canvas pixel (cx, cy) to world (x, y) meters."""
        col = self.crop[0] + cx / self.zoom
        row = self.crop[1] + cy / self.zoom
        return self.map.pixel_to_world(col, row)

    def world_to_canvas(self, x, y):
        """Convert world (x, y) meters to a canvas pixel (cx, cy)."""
        col, row = self.map.world_to_pixel(x, y)
        return (col - self.crop[0]) * self.zoom, (row - self.crop[1]) * self.zoom

    def on_press(self, event):
        """Start a new stroke at the click point."""
        if self.map is None:
            return
        self.strokes.append([(event.x, event.y)])
        self.waypoints = []
        self.bad_points = []

    def on_drag(self, event):
        """Extend the current stroke and draw the new segment."""
        if self.map is None or not self.strokes:
            return
        stroke = self.strokes[-1]
        last = stroke[-1]
        if math.hypot(event.x - last[0], event.y - last[1]) < 2:
            return
        stroke.append((event.x, event.y))
        self.canvas.create_line(
            last[0],
            last[1],
            event.x,
            event.y,
            fill="#d62728",
            width=3,
            capstyle=tk.ROUND,
            tags="stroke",
        )

    def undo(self):
        """Remove the last drawn stroke."""
        if self.strokes:
            self.strokes.pop()
        self.waypoints = []
        self.bad_points = []
        self.redraw()

    def clear(self):
        """Remove every drawn stroke and the goal marker."""
        self.strokes = []
        self.waypoints = []
        self.bad_points = []
        self.goal_click = None
        self.redraw()

    def on_click_goal(self, event):
        """Set the A* goal to the right-clicked point."""
        if self.map is None:
            return
        self.goal_click = self.canvas_to_world(event.x, event.y)
        self.redraw()
        self.set_message(
            f"Goal set at ({self.goal_click[0]:.2f}, {self.goal_click[1]:.2f}). "
            'Press "Plan (A*)" to route there from the robot\'s current position.'
        )

    def redraw(self):
        """
        Repaint the map, every drawn stroke, the planned waypoints (and
        any flagged as too close to an obstacle), and the goal marker.
        """
        if self.map is None:
            return
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
        joined = [p for stroke in self.strokes for p in stroke]
        for a, b in zip(joined, joined[1:]):
            self.canvas.create_line(
                a[0],
                a[1],
                b[0],
                b[1],
                fill="#d62728",
                width=3,
                capstyle=tk.ROUND,
                tags="stroke",
            )
        if joined:
            x, y = joined[0]
            self.canvas.create_oval(
                x - 6, y - 6, x + 6, y + 6, fill="#2ca02c", outline=""
            )
            x, y = joined[-1]
            self.canvas.create_rectangle(
                x - 6, y - 6, x + 6, y + 6, fill="#d62728", outline=""
            )
        for wx, wy in self.waypoints:
            x, y = self.world_to_canvas(wx, wy)
            self.canvas.create_oval(
                x - 2, y - 2, x + 2, y + 2, fill="#ff7f0e", outline=""
            )
        for wx, wy in self.bad_points:
            x, y = self.world_to_canvas(wx, wy)
            self.canvas.create_oval(
                x - 8, y - 8, x + 8, y + 8, outline="#ff00ff", width=3
            )
        if self.goal_click is not None:
            x, y = self.world_to_canvas(*self.goal_click)
            self.canvas.create_oval(
                x - 6, y - 6, x + 6, y + 6, outline="#9467bd", width=3, tags="goal"
            )

    def plan(self):
        """
        Turn the drawn strokes into a smoothed, evenly-spaced world-frame
        waypoint list.
        """
        joined = [p for stroke in self.strokes for p in stroke]
        world = [self.canvas_to_world(cx, cy) for cx, cy in joined]
        return smooth_path(resample_path(world, self.node.waypoint_spacing))

    def _follow_if_valid(self, waypoints, label):
        """
        checks waypoints against the map for wall or obstacles and.

        updates self.waypoints and self.bad_points and redraws either way
        starts following if the whole path is clear.

        Returns True if it was accepted and handed to the robot
        False if any waypoint was too close to an obstacle
        """
        radius = self.node.robot_radius
        states = [self.map.cell_state_at(x, y, radius) for x, y in waypoints]
        self.bad_points = [w for w, s in zip(waypoints, states) if s == "occupied"]
        self.waypoints = waypoints
        self.redraw()
        if self.bad_points:
            return False
        unknown = states.count("unknown")
        length = sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(waypoints, waypoints[1:])
        )
        note = f" ({unknown} waypoints are in unscanned area)" if unknown else ""
        self.set_message(f"{label}: {len(waypoints)} waypoints, {length:.1f} m{note}")
        self.node.follow(waypoints)
        return True

    def go(self):
        """
        Follow the drawn path, or if it crosses an obstacle, fall back to
        an A*-planned route to the same destination.
        """
        if self.map is None:
            return
        waypoints = self.plan()
        if len(waypoints) < 2:
            self.set_message("Draw a path first (click and drag on the map).")
            return
        if self._follow_if_valid(waypoints, "Going"):
            return

        # the drawn path crosses a wall/obstacle -- fall back to A*, aiming
        # for the same destination the drawing was headed toward, from
        # wherever the robot actually is right now
        n_bad = len(self.bad_points)
        goal_xy = waypoints[-1]
        if not self.node.have_odom:
            self.set_message(
                f"{n_bad} waypoint(s) (circled) cross a wall or obstacle, and "
                "there's no odometry yet for A* to replan from. Redraw further "
                "from walls instead."
            )
            return
        self.set_message(
            f"{n_bad} waypoint(s) (circled) cross a wall or obstacle -- "
            "replanning with A* to the same destination..."
        )
        start_xy = (self.node.x, self.node.y)
        world_path = plan_world_path(
            self.map, start_xy, goal_xy, robot_radius=self.node.robot_radius
        )
        if world_path is None:
            self.set_message(
                "A* also found no valid path to that destination -- "
                "try drawing to a different point."
            )
            return
        astar_waypoints = smooth_path(
            resample_path(world_path, self.node.waypoint_spacing)
        )
        if not self._follow_if_valid(astar_waypoints, "A* (auto-replanned)"):
            self.set_message(
                f"{len(self.bad_points)} waypoint(s) (circled) are too close to "
                "a wall even after A* replanning -- try a different destination."
            )

    def plan_astar(self):
        """
        Plan and follow an A* route from the robot's current position to
        the right-clicked goal.
        """
        if self.map is None:
            return
        if self.goal_click is None:
            self.set_message("Right-click a point on the map to set a goal first.")
            return
        if not self.node.have_odom:
            self.set_message(
                "No odometry yet -- can't plan from the robot's current position."
            )
            return

        start_xy = (self.node.x, self.node.y)
        world_path = plan_world_path(
            self.map, start_xy, self.goal_click, robot_radius=self.node.robot_radius
        )
        if world_path is None:
            self.set_message("A* found no path to that goal (blocked or unreachable).")
            return

        waypoints = smooth_path(resample_path(world_path, self.node.waypoint_spacing))
        if not self._follow_if_valid(waypoints, "A*"):
            self.set_message(
                f"{len(self.bad_points)} waypoint(s) (circled) are too close to a "
                "wall after smoothing -- try a different goal."
            )

    def stop(self):
        """Cancel following, without leaving PATH FOLLOWING mode."""
        self.node.cancel()
        self.set_message("Stopped.")

    def set_message(self, text):
        """Set the window's bottom status message."""
        self.message.config(text=text)

    def tick(self):
        """
        Refresh the status line and redraw the robot marker at its
        current pose; reschedules itself.
        """
        self.status.config(
            text=f"{self.node.status_text()}   "
            f"robot=({self.node.x:.2f}, {self.node.y:.2f})"
        )
        self.canvas.delete("robot")
        if self.map is not None and self.node.have_odom:
            x, y = self.world_to_canvas(self.node.x, self.node.y)
            r = self.node.robot_radius / self.map.resolution * self.zoom
            hx = x + r * math.cos(self.node.yaw)
            hy = y - r * math.sin(self.node.yaw)
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r, outline="#1f77b4", width=2, tags="robot"
            )
            self.canvas.create_line(x, y, hx, hy, fill="#1f77b4", width=2, tags="robot")
        self.root.after(self.REFRESH_MS, self.tick)

    def update_ui(self):
        """Processes one iteration of Tkinter events without blocking execution."""
        try:
            self.root.update_idletasks()
            self.root.update()
            return True
        except tk.TclError:
            # Handle cases where the window was closed manually via window manager [X]
            return False

    def show(self):
        """Reveal the drawing window."""
        self.root.deiconify()

    def hide(self):
        """Hide the drawing window without destroying it."""
        self.root.withdraw()

    def close(self):
        """
        The window's own [X] button: cancel the path and tell fsm_node
        to leave PATH FOLLOWING, same as switching modes any other way.
        """
        self.node.cancel()
        self.node.send_fsm_command("g")
        self.hide()


class ControlPanel:
    """
    Always-visible window: buttons to switch fsm_node's mode (the same.

    t/m/g/p switches its own keyboard listener and /fsm_command accept), a
    live current_mode line, and a scrolling log of past mode changes. Owns
    the one persistent Tk root for this process; the path-drawing window
    (PathPainter) is a Toplevel under it that this panel shows and hides as
    current_mode changes, rather than creating and destroying it each time.

    This exists because fsm_node's own keyboard control only works when it
    has a real terminal (not true under ros2 launch), and the path-drawing
    window itself only exists once already in PATH FOLLOWING -- so neither
    one can be where you switch *into* that mode. This panel can, because
    it's up the whole time regardless of mode.
    """

    REFRESH_MS = 200
    DRIVE_MS = 100
    MODES = (
        ("Teleop Scan", "t"),
        ("Drive Square", "m"),
        ("Wall Follow", "g"),
        ("Path Following", "p"),
    )
    # arrow key -> (linear, angular) sign; held keys are summed and scaled
    DRIVE_KEYS = {
        "Up": (1.0, 0.0),
        "Down": (-1.0, 0.0),
        "Left": (0.0, 1.0),
        "Right": (0.0, -1.0),
    }

    def __init__(self, node, ui_queue):
        """
        Build the mode buttons, status line, drive area, and mode-history
        log, and create the (initially hidden) drawing window under this
        panel's root.
        """
        self.node = node
        self.ui_queue = ui_queue
        self.held_keys = set()
        self.linear_speed = 0.15
        self.angular_speed = 0.6

        self.root = tk.Tk()
        self.root.title("Neato FSM control panel")

        bar = tk.Frame(self.root)
        bar.pack(side=tk.TOP, fill=tk.X, padx=4, pady=4)
        for label, key in self.MODES:
            tk.Button(bar, text=label, width=14,
                     command=lambda k=key: self.node.send_fsm_command(k)
                     ).pack(side=tk.LEFT, padx=2)

        self.status = tk.Label(self.root, text="mode: (waiting for fsm_node)",
                               anchor="w", font=("TkDefaultFont", 11, "bold"))
        self.status.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(4, 0))
        tk.Label(self.root, text="Drive Square only takes effect while in Teleop "
                 "Scan; Path Following opens the drawing window below.",
                 anchor="w", fg="#555555").pack(side=tk.TOP, fill=tk.X, padx=6)

        drive_bar = tk.Frame(self.root)
        drive_bar.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(8, 0))
        tk.Label(drive_bar, text="Drive (click here, then use the arrow "
                 "keys -- switches to Teleop Scan automatically):",
                 anchor="w").pack(side=tk.TOP, fill=tk.X)
        self.drive_status = tk.Label(drive_bar, text="not driving", fg="#555555", anchor="w")
        self.drive_status.pack(side=tk.LEFT)
        tk.Button(drive_bar, text="Save Map", command=self.save_map).pack(side=tk.RIGHT)
        self.save_status = tk.Label(drive_bar, text="", fg="#1a7a1a", anchor="e")
        self.save_status.pack(side=tk.RIGHT, padx=8)
        # a focus-able target for the arrow keys, separate from the mode
        # buttons above so Tab/click doesn't accidentally leave it on a
        # button (where Space/Return would re-trigger that button instead)
        self.drive_target = tk.Frame(self.root, height=2, bg="#cccccc",
                                     highlightthickness=1, highlightbackground="#999999")
        self.drive_target.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(2, 6))
        self.drive_target.focus_set()
        for key in self.DRIVE_KEYS:
            self.root.bind(f"<KeyPress-{key}>", lambda e, k=key: self.on_drive_press(k))
            self.root.bind(f"<KeyRelease-{key}>", lambda e, k=key: self.on_drive_release(k))
        self.root.bind("<space>", lambda e: self.held_keys.clear())

        tk.Label(self.root, text="Mode history:", anchor="w").pack(
            side=tk.TOP, fill=tk.X, padx=6, pady=(6, 0))
        log_frame = tk.Frame(self.root)
        log_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))
        scrollbar = tk.Scrollbar(log_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log = tk.Listbox(log_frame, height=8, width=48, yscrollcommand=scrollbar.set)
        self.log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.log.yview)

        self.painter = PathPainter(node, master=self.root)
        self.logged_through = 0

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(self.REFRESH_MS, self.tick)
        self.root.after(self.DRIVE_MS, self.drive_tick)

    def on_drive_press(self, key):
        """
        Record an arrow key as held, switching to TELEOP SCAN first if
        this is the first key pressed while in some other mode.
        """
        if not self.held_keys and self.node.current_mode != "TELEOP SCAN":
            self.node.send_fsm_command("t")
        self.held_keys.add(key)

    def on_drive_release(self, key):
        """Stop treating an arrow key as held."""
        self.held_keys.discard(key)

    def drive_tick(self):
        """
        Composes the currently-held arrow keys into one Twist and
        publishes it, the same way teleop_scan.py's own 0.1s timer keeps
        re-sending whatever the last keypress set -- so releasing every key
        stops the robot instead of leaving a stale command in flight.
        """
        linear = angular = 0.0
        for key in self.held_keys:
            lin, ang = self.DRIVE_KEYS[key]
            linear += lin
            angular += ang
        self.node.drive_teleop(linear * self.linear_speed, angular * self.angular_speed)
        self.drive_status.config(
            text=f"driving: linear={linear * self.linear_speed:.2f} m/s "
                 f"angular={angular * self.angular_speed:.2f} rad/s"
            if self.held_keys else "not driving")
        self.root.after(self.DRIVE_MS, self.drive_tick)

    def save_map(self):
        """
        Tell teleop_scan to save its map now, and show a confirmation
        that clears itself after a few seconds.
        """
        self.node.save_teleop_map()
        # a separate label from drive_status, which drive_tick overwrites every 100ms
        self.save_status.config(text=f"saved {datetime.now().strftime('%H:%M:%S')}")
        self.root.after(3000, lambda: self.save_status.config(text=""))

    def tick(self):
        """
        Refresh the status line, append any new mode-history entries to
        the log, and show/hide the drawing window on a pending mode change;
        reschedules itself.
        """
        self.status.config(text=f"mode: {self.node.current_mode}")

        history = self.node.mode_history
        for stamp, mode in list(history)[self.logged_through:]:
            self.log.insert(tk.END, f"{stamp}  {mode}")
            self.log.see(tk.END)
        self.logged_through = len(history)

        try:
            cmd = self.ui_queue.get_nowait()
            if cmd == "SHOW_UI":
                self.painter.show()
            elif cmd == "HIDE_UI":
                self.painter.hide()
        except queue.Empty:
            pass

        self.root.after(self.REFRESH_MS, self.tick)

    def on_close(self):
        """
        The panel's own [X] button: cancel any path in progress, switch
        back to wall following, and exit the Tk main loop.
        """
        self.node.cancel()
        self.node.send_fsm_command("g")
        self.root.quit()

    def run(self):
        """Run the Tk main loop until the panel is closed."""
        self.root.mainloop()


def main(args=None):
    """
    Initialize rclpy, run the control panel until it's closed, then shut
    down. Exits with an error if no saved map can be loaded.
    """
    # let Ctrl-C raise here so the robot still gets a zero velocity before rclpy shuts down
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    ui_queue = queue.Queue()
    node = PathFollower(ui_queue)
    # the robot must start where it did during teleop_scan, or the map and odom won't line up
    try:
        SavedMap.load(node.map_file)
    except OSError:
        print(
            f"No map found at {node.map_file}.\n"
            "Run `ros2 run ros_behaviors_fsm teleop_scan` first, drive around "
            "the room, and press m to save it."
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    panel = ControlPanel(node, ui_queue)
    try:
        panel.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel()
        rclpy.shutdown()
        spin_thread.join()
        node.destroy_node()


if __name__ == "__main__":
    main()
