# RoboBehaviors and Finite State Machines Project

Author: Aditi Lagisetty

For Olin ENGR3590 Computational Introduction to Robotics

## Status / Remaining Work

This section tracks what's left before this is submission-ready. Remove it
once everything below is done.

**Required, not yet done:**
- [ ] Finish `collision_avoidance.py`'s tuning -- the logic itself (hard-stop
      backstop, potential-fields steering, side-swipe handling via the
      repulsion-magnitude slowdown) is all implemented, but `k_attractive`,
      `k_repulsive`, and `k_steer` are still untested placeholder values --
      needs real Gazebo testing to tune.
- [ ] Fix a real bug in `wall_follower.py` -- it checks
      `msg.ranges[270] == self.distance_from_wall` (exact float equality
      against a live sensor reading), which will essentially never be true.
      This is likely why the commit history notes "simulation isn't
      working." Also still needs the `visualization_msgs/Marker` showing the
      detected wall (**required** by the assignment for this behavior).
- [x] ~~Write a self-designed behavior~~ -- superseded the original
      `line_following.py` idea (deleted, was an unused empty stub) with a
      much bigger one: `teleop_scan.py` (drive around, build an
      occupancy-grid map) + `room_map.py` (the map itself) + `a_star.py`
      (search that map for a path) + `path_following.py` (a Tkinter GUI to
      draw a path by hand *or* right-click a goal and have A* plan it, then
      drive the result with pure-pursuit control, pausing on
      bump/e-stop/obstacle). This is now wired into
      `finite_state_controller.py` as a `PATH_FOLLOWING` state (see the FSM
      section below) -- satisfies the "at least 1 self-designed behavior"
      requirement on its own.
- [ ] Wire `wall_follower` into `finite_state_controller.py` as a real state
      (`WALL_FOLLOWING` is currently still a TODO stub there -- everything
      else, including the new `PATH_FOLLOWING` state, is wired up).
- [ ] Record every bag file in `bags/` (currently empty):
      `test_drive.bag`, `drive_square_demo.bag`,
      `collision_avoidance_demo.bag`, `wall_follower_demo.bag`,
      `path_following_demo.bag`, `finite_state_controller_demo.bag`.
      Use `ros2 bag record /accel /bump /odom /cmd_vel /scan /stable_scan
      /projected_stable_scan /tf /tf_static -o <name>` (see "How To Run").
- [ ] Fill in the still-TODO write-up sections below: Behaviors 2-4's
      description/implementation/design-decisions/demo, the FSM's
      implementation details + demonstration + a state-transition diagram,
      and Learning Objectives.
- [ ] Add photos/gifs/videos of each behavior (the assignment explicitly
      wants this to be portfolio-quality, not just text).
- [ ] Test everything on the **physical Neato** -- everything so far has
      only been run in the Gazebo simulator, and the assignment requires
      working robot code by the end, not just a working simulation. The
      `PATH_FOLLOWING` state chain in particular (map -> A* -> follow ->
      FSM handoff) has never been run live at all, only unit-tested offline.

## Project Overview

TODO: overall description of the project, motivation (link to the
assignment prompt), key design choices, and a summary of key takeaways.

## Individual Behaviors

In this section you will provide the details of each individual behavior you
created / is part of this repository. At a minimum, you should have an
emergency stop, driving in a shape, and wall-following. Any additional
behaviors you created as part of your finite-state-machine should also be
described here.

### Behavior 1: Drive in a Square

- **What it does:** [drive_square.py](ros_behaviors_fsm/drive_square.py)
  drives the Neato through a 1m x 1m square path, using odometry feedback
  (not pure timing) to know when it's actually turned 90 degrees or driven
  1m. It also subscribes to an `estop` topic and to `scan` so it can stop
  (and later resume) if something is within `stop_distance` in front of the
  robot.
- **Implementation details:** subscribes to `estop` (`std_msgs/Bool`),
  `scan` (`sensor_msgs/LaserScan`), and `odom` (`nav_msgs/Odometry`);
  publishes to `cmd_vel` (`geometry_msgs/Twist`). The driving logic runs on
  its own thread, separate from the callbacks that watch for the e-stop
  condition (see the threading discussion below). Turning/driving use
  proportional control on the remaining angle/distance (see
  `turn_left`/`drive_forward`) rather than a constant speed until a hard
  cutoff, since Gazebo's diff-drive plugin has a real acceleration limit
  (`max_wheel_acceleration` in the Neato SDF) that causes noticeable
  overshoot with a bang-bang stop. An explicit `settle()` pause after each
  motion lets residual velocity fully decay before the next one starts, so
  turns are clean in-place pivots rather than arcs.
- **Design decisions:**
  - **Single- vs. multi-threaded:** Before adapting [drive_square.py](ros_behaviors_fsm/drive_square.py)
    into a dedicated Thread running the drive sequence (multi-threaded), we
    compared it against the class's timer-callback-based single-threaded
    sample. We chose the multi-threaded approach because the drive sequence
    needs to react to an asynchronous e-stop condition (bump/obstacle)
    coming in on a separate callback -- a pure timer-based loop can't be
    interrupted mid-segment without extra bookkeeping, whereas a
    `threading.Event`-style shared flag between the driving thread and the
    ROS callbacks handles that naturally.
  - Annotating the two samples surfaced real bugs in the originals, since
    fixed: the single-threaded sample never stopped after the fourth side
    (nothing checked `turns_executed` to end the loop), and the
    multi-threaded sample's e-stop could never be cleared once triggered.
  - Time-based timing alone (the class's original samples) reliably
    undershot the target in Gazebo, because the sim's acceleration limit
    means the robot doesn't reach the assumed speed instantly. Switching to
    odometry feedback plus proportional control (rather than a fixed speed
    + sleep) fixed this -- confirmed against RViz's rendering of `/odom` and
    TF, which traces a clean square even in cases where the Gazebo 3D
    viewport's own rendering looked visually off (a rendering/camera-angle
    issue in the viewport itself, not a control bug).
- **Demo:** TODO -- add a gif/video and link to `bags/drive_square_demo`.

### Behavior 2: Collision Avoidance + Obstacle Avoidance

TODO. Starting point: [collision_avoidance.py](ros_behaviors_fsm/collision_avoidance.py)
-- combines what would otherwise be two separate behaviors into one node:
reactive stopping (bump-triggered / critically-close-range triggered, from
the day 3 e-stop approaches taught in class) as a hard-stop safety backstop,
plus continuous potential-fields steering (constant forward attraction +
per-scan-point repulsion, summed into a steering direction) for anything
farther out that the robot can reroute around instead of just halting for.
Subscriptions/publishers (including an optional force-vector
`visualization_msgs/Marker` for debugging in RViz) are wired up; the
repulsive-force sum and the force-to-steering-command conversion are left as
TODOs to design and implement.

### Behavior 3: Wall Following

TODO. Starting point: [wall_follower.py](ros_behaviors_fsm/wall_follower.py)
(skeleton only -- proportional control logic not yet implemented). Still
needs the required wall-detection `visualization_msgs/Marker`.

### Behavior 4: Mapping + A* Path Planning + Path Following (self-designed)

This is the behavior meant to satisfy the assignment's "at least one
behavior outside class activities" requirement. It's actually four pieces
working together:

- [teleop_scan.py](ros_behaviors_fsm/teleop_scan.py): drive the robot by
  keyboard while it builds an occupancy-grid map from the lidar (log-odds
  Bayesian updates in [room_map.py](ros_behaviors_fsm/room_map.py)), saved
  to disk as a standard PGM+YAML map pair.
- [a_star.py](ros_behaviors_fsm/a_star.py): a global path planner over that
  saved map -- inflates obstacles by the robot's radius (so found paths
  keep its whole body clear of walls, not just its center point), then runs
  A* search to find the shortest obstacle-free route between two points.
- [path_following.py](ros_behaviors_fsm/path_following.py): a Tkinter GUI
  (`PathPainter`) showing the saved map, where you can either draw a path by
  hand or right-click to set a goal and click "Plan (A*)" to have it planned
  automatically. Either way, the resulting waypoints get resampled/smoothed
  and driven using pure-pursuit steering control, pausing (not stopping
  outright) on bump/e-stop/obstacle-close and resuming once clear. If a
  hand-drawn path crosses an obstacle, it automatically falls back to
  planning an A* route to the same intended destination instead of just
  rejecting the drawing.
- TODO: write up the actual design decisions here once tested live (e.g.,
  why pure-pursuit for path following, why occupancy-grid + A* rather than
  a different mapping/planning approach, what the log-odds map update
  parameters mean) -- everything above has been unit-tested offline (fake
  maps, doorways of known width) but never run against the real sim/robot.

**Demo:** TODO -- add a gif/video and link to `bags/path_following_demo`.

## Finite State Machine

### Overall Design

- **States:** `DRIVE_SQUARE` (default), `COLLISION_AVOIDANCE` (entered on
  bump or close-range obstacle -- also handles steering around obstacles
  that aren't critically close, and side-swipe risks), `WALL_FOLLOWING`
  (entered when a wall is detected nearby; steering law still TODO),
  `PATH_FOLLOWING` (entered whenever the separate `path_following.py` node
  is actively driving toward a goal or paused mid-path).
- **Transitions:** `DRIVE_SQUARE` -> `COLLISION_AVOIDANCE` on bump or a
  close-range front reading; back to `DRIVE_SQUARE` once clear.
  `DRIVE_SQUARE` -> `WALL_FOLLOWING` when a wall is detected within
  `wall_detect_distance`; back once it's no longer detected.
  Any state -> `PATH_FOLLOWING` whenever `path_following.py` reports it's
  `following` or `paused` (see coding strategy below); back to
  `DRIVE_SQUARE` once it goes idle/done. `PATH_FOLLOWING` takes priority
  over everything else -- if a path is actively being followed, the FSM
  defers to it regardless of what state it would otherwise be in.

### Coding Strategy

Three of the four states (`DRIVE_SQUARE`, `COLLISION_AVOIDANCE`,
`WALL_FOLLOWING`) are re-implemented directly inside
`finite_state_controller.py` as methods on one `Node` -- a single-file,
single-process design, since these three all just need direct sensor
readings and a `cmd_vel` command each tick.

`PATH_FOLLOWING` is different: it **chains a separate, independent node
together with the state machine**, rather than reimplementing its logic
inline. `path_following.py` keeps running exactly as it does standalone --
its own `cmd_vel` publisher, its own bump/e-stop/obstacle pause logic, its
own GUI. It additionally publishes a small `path_following_status` topic
(`'idle'`/`'following'`/`'paused'`/`'done'`), which `finite_state_controller.py`
subscribes to in order to know when to be in the `PATH_FOLLOWING` state.
While in that state, the FSM's `handle_path_following()` deliberately
**publishes nothing at all** -- the point is to hand off control entirely,
since having both nodes react to the same sensors and publish competing
`cmd_vel` commands at once would be worse than either one alone. This
matches the "chain parallel nodes together" strategy the assignment
explicitly calls out as a valid way to structure an FSM, and was a
deliberate choice over reimplementing `path_following.py`'s pure-pursuit
steering and Tkinter GUI inline, given how architecturally different that
node already is from the other three states.

**Capabilities and limitations:** the FSM can drive a square, safely stop
or reroute around obstacles, and hand off to a fully-planned/hand-drawn
path -- but the hand-off itself is not triggered by anything the robot
senses in its environment (it's triggered by a human interacting with
`path_following.py`'s GUI), which is a real limitation against the
assignment's "each transition should be some condition you can reliably
detect in the environment" guidance -- worth being upfront about in the
final writeup rather than glossing over.

### Implementation Details

TODO -- pointers to relevant code once wall-following is wired up and
everything's been tested live.

### Demonstration

TODO -- add a gif/video and link to `bags/finite_state_controller_demo`.

## Learning Objectives and Final Takeaways

TODO.

## How To Run

1. Clone this repo into the `src/` directory of a ROS2 workspace.
2. From the workspace root: `colcon build --packages-select ros_behaviors_fsm`
3. `source install/setup.bash`
4. Run an individual behavior, e.g. `ros2 run ros_behaviors_fsm drive_square`
5. Record a demo: `ros2 bag record /accel /bump /odom /cmd_vel /scan
   /stable_scan /projected_stable_scan /tf /tf_static -o bags/<name>`
6. Play back a recorded run: `ros2 bag play bags/<bag-name> --clock`
