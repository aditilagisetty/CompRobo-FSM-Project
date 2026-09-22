import math
import queue
import sys
import time
import tkinter as tk
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
from std_msgs.msg import Bool, String

from .a_star import plan_world_path
from .angle_helpers import euler_from_quaternion
from .room_map import DEFAULT_MAP_FILE, SavedMap

TURN_IN_PLACE_ANGLE = math.radians(50)
FRONT_CONE = math.radians(20)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def resample_path(points, spacing):
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
    pts = np.asarray(points, dtype=float)
    for _ in range(passes):
        if len(pts) < 3:
            break
        pts[1:-1] = (pts[:-2] + pts[1:-1] + pts[2:]) / 3.0
    return [tuple(p) for p in pts.tolist()]


class PathFollower(Node):
    def __init__(self, ui_queue: Queue):
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

        self.create_subscription(String, "current_mode", self.mode_callback, 10)

        self.vel_pub = self.create_publisher(Twist, "cmd_vel_path_following", 10)
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "drawn_path", path_qos)
        # lets finite_state_controller.py know whether this node currently
        # owns cmd_vel (active or paused-but-not-done, vs idle/done), so it
        # knows when to step back and when to resume its own driving.
        self.status_pub = self.create_publisher(String, "path_following_status", 10)
        self.create_subscription(Odometry, "odom", self.process_odom, 10)
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.create_subscription(Bump, "bump", self.process_bump, 10)
        self.create_subscription(Bool, "estop", self.handle_estop, 10)
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.1, self.publish_status)

    def mode_callback(self, msg):
        if msg.data == "PATH FOLLOWING":
            self.ui_queue.put("SHOW_UI")
        else:
            self.ui_queue.put("HIDE_UI")

    def process_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def process_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=float)
        # ranges[0] is the front and beams go counterclockwise
        angles = np.arange(len(ranges)) * msg.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        in_cone = np.abs(angles) < FRONT_CONE
        r = ranges[in_cone]
        r = r[np.isfinite(r) & (r > msg.range_min)]
        self.obstacle_close = bool(len(r) and r.min() < self.stop_distance)

    def process_bump(self, msg):
        if msg.left_front or msg.left_side or msg.right_front or msg.right_side:
            self.last_bump_time = time.monotonic()
            self.bumped = True

    def check_bump_timeout(self):
        if self.bumped and time.monotonic() - self.last_bump_time > self.bump_timeout_sec:
            self.bumped = False

    def handle_estop(self, msg):
        self.manual_estop = bool(msg.data)

    def follow(self, waypoints):
        with self.lock:
            self.path = list(waypoints)
            self.idx = 0
            self.state = "following"
        self.publish_path(waypoints)
        self.get_logger().info(f"following a path of {len(waypoints)} waypoints")

    def cancel(self):
        with self.lock:
            self.state = "idle"
            self.path = []
        self.drive(0.0, 0.0)

    def publish_path(self, waypoints):
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
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)

    def status_text(self):
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
        with self.lock:
            state = self.state
        self.status_pub.publish(String(data=state))

    def control_loop(self):
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
        path = self.path

        def dist(i):
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
    MAX_WIDTH = 1000
    MAX_HEIGHT = 700
    REFRESH_MS = 100

    def __init__(self, node):
        self.node = node
        self.map = None
        self.strokes = []
        self.waypoints = []
        self.bad_points = []
        self.goal_click = None

        self.root = tk.Tk()
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

    def reload_map(self):
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
        col = self.crop[0] + cx / self.zoom
        row = self.crop[1] + cy / self.zoom
        return self.map.pixel_to_world(col, row)

    def world_to_canvas(self, x, y):
        col, row = self.map.world_to_pixel(x, y)
        return (col - self.crop[0]) * self.zoom, (row - self.crop[1]) * self.zoom

    def on_press(self, event):
        if self.map is None:
            return
        self.strokes.append([(event.x, event.y)])
        self.waypoints = []
        self.bad_points = []

    def on_drag(self, event):
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
        if self.strokes:
            self.strokes.pop()
        self.waypoints = []
        self.bad_points = []
        self.redraw()

    def clear(self):
        self.strokes = []
        self.waypoints = []
        self.bad_points = []
        self.goal_click = None
        self.redraw()

    def on_click_goal(self, event):
        if self.map is None:
            return
        self.goal_click = self.canvas_to_world(event.x, event.y)
        self.redraw()
        self.set_message(
            f"Goal set at ({self.goal_click[0]:.2f}, {self.goal_click[1]:.2f}). "
            'Press "Plan (A*)" to route there from the robot\'s current position.'
        )

    def redraw(self):
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
        joined = [p for stroke in self.strokes for p in stroke]
        world = [self.canvas_to_world(cx, cy) for cx, cy in joined]
        return smooth_path(resample_path(world, self.node.waypoint_spacing))

    def _follow_if_valid(self, waypoints, label):
        """checks waypoints against the map for wall or obstacles and
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
        self.node.cancel()
        self.set_message("Stopped.")

    def set_message(self, text):
        self.message.config(text=text)

    def tick(self):
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

    def close(self):
        self.node.cancel()
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        self.root.mainloop()


def main(args=None):
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

    painter = None

    try:
        while rclpy.ok():
            try:

                cmd = ui_queue.get(timeout=0.2)

                if cmd == "SHOW_UI" and painter is None:
                    node.get_logger().info(
                        "Path following mode active. Launching UI..."
                    )
                    painter = PathPainter(node)
                    painter.run()
                    painter = None

                elif cmd == "HIDE_UI" and painter is not None:
                    node.get_logger().info(
                        "Path following mode inactive. Closing UI..."
                    )
                    painter.close()
                    painter = None

            except queue.Empty:
                pass

    except KeyboardInterrupt:
        pass
    finally:
        if painter is not None:
            painter.close()
        node.cancel()
        rclpy.shutdown()
        spin_thread.join()
        node.destroy_node()


if __name__ == "__main__":
    main()
