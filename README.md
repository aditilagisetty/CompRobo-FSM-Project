# RoboBehaviors and Finite State Machines Project

Author: Aditi Lagisetty

For Olin ENGR3590 Computational Introduction to Robotics

## Status / Remaining Work

This section tracks what's left before this is submission-ready. Remove it
once everything below is done.

**Required, not yet done:**
- [ ] Finish `collision_avoidance.py` -- this now combines two things: a
      hard-stop safety backstop (bump, proximity, side-swipe trajectory --
      still TODO) and continuous potential-fields steering around obstacles
      that aren't critically close yet (also still TODO: the repulsive-force
      sum and the force-to-steering conversion).
- [ ] Finish `wall_follower.py` -- the proportional steering law itself is a
      TODO. Also needs the `visualization_msgs/Marker` showing the detected
      wall (**required** by the assignment for this behavior, not optional).
- [ ] Write `line_following.py` (currently an empty file) -- this is the
      self-designed behavior. The assignment requires the FSM to combine
      **at least 3 behaviors, at least 1 of which is self-designed / outside
      the in-class activities**. `drive_square`, `collision_avoidance`, and
      `wall_follower` are all in-class behaviors (even with obstacle
      avoidance folded into `collision_avoidance`), so `line_following`
      needs to actually get built and wired into the FSM to meet that
      requirement.
- [ ] Wire `wall_follower` and `line_following` into
      `finite_state_controller.py` as real states (currently only
      `DRIVE_SQUARE` has real driving logic and `COLLISION_AVOIDANCE` just
      hard-stops; `WALL_FOLLOWING` is a TODO stub and there's no
      line-following state yet at all).
- [ ] Record every bag file in `bags/` (currently empty):
      `test_drive.bag`, `drive_square_demo.bag`,
      `collision_avoidance_demo.bag`, `wall_follower_demo.bag`,
      `line_following_demo.bag`, `finite_state_controller_demo.bag`.
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
      working robot code by the end, not just a working simulation.

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

### Behavior 4: Line Following (self-designed)

This is the behavior meant to satisfy the assignment's "at least one
behavior outside class activities" requirement.

TODO -- not started yet ([line_following.py](ros_behaviors_fsm/line_following.py)
is currently an empty file).

## Finite State Machine

### Overall Design

TODO -- flesh out once behaviors 2-4 are implemented and actually wired into
the FSM (currently only `DRIVE_SQUARE` has real behavior; the others are
placeholders/TODOs -- see `finite_state_controller.py`). Draft design:

- **States:** `DRIVE_SQUARE` (default), `COLLISION_AVOIDANCE` (entered on
  bump or close-range obstacle -- now also handles steering around obstacles
  that aren't critically close), `WALL_FOLLOWING` (entered when a wall is
  detected nearby). TODO: add a `LINE_FOLLOWING` state once
  `line_following.py` is implemented, plus its transition criteria.
- **Transitions:** TODO -- define the actual sensor thresholds/logic for
  each transition listed in the skeleton file's docstring.

### Implementation Details

TODO -- pointers to relevant code once the FSM is implemented.

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
