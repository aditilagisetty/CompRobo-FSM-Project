# RoboBehaviors and Finite State Machines Project

**Author Names:** Aditi, Duc, and Akil

**Course:** Olin ENGR3590 Computational Introduction to Robotics

**Assignment:** [RoboBehaviors and Finite State Machines](https://comprobo26.github.io/assignments/warmup_project)

## Project Overview

This project programs a Neato robot (ROS 2 Jazzy, Gazebo Harmonic) to run several behaviors and switch between them with a finite state machine. The assignment asks for a set of reactive behaviors (stopping for obstacles, driving a shape, following a wall, and at least one of our own design) and a state machine that chooses among them. This repository is our solution. Every node has been run live in the Gazebo simulator and is demonstrated by a bag in `bags/` (seven in total). Two different finite-state-machine designs exist -- `finite_state_controller.py` (one node, sensor callbacks and per-state handlers) and `fsm_node.py` (a topic-mux gateway) -- and both have been run end to end; neither has been tested on the physical robot. [What We Verified](#what-we-verified) lists exactly what was checked and how.

The behaviors are:

| Behavior | File | Category |
|---|---|---|
| Drive a 1 m square | `drive_square.py` | Fundamental (in class) |
| Collision avoidance with obstacle steering | `collision_avoidance.py` | Basic (in class) |
| Wall following | `wall_follower.py` | Advanced (in class) |
| Mapping, A* planning and path following | `teleop_scan.py`, `room_map.py`, `a_star.py`, `path_following.py` | **Self-designed** |

The finite state machine in `finite_state_controller.py` combines all four. Three of its states are implemented inside the FSM node itself; the fourth (path following) is a separate node that the FSM hands control to.

Key design choices, each explained in the sections below:

- **Closed-loop motion instead of timing.** The simulator's wheel-acceleration limit makes timed "drive, then stop" commands drift, so `drive_square.py` uses odometry feedback and proportional control.
- **Potential fields for collision avoidance.** The robot steers around obstacles it can see coming and keeps a hard stop only as a last resort.
- **A separate node for path following, chained into the FSM.** It has its own GUI and its own safety pausing, so the FSM steps aside while it drives instead of competing with it for `cmd_vel`.
- **A timed escape hatch alongside a distance threshold.** `fsm_node.py`'s reactive states could get permanently stuck right at a threshold boundary (see the FSM section); a "how long has this been true" timeout, checked independent of the current state, breaks that without changing behavior anywhere else.

**Summary of takeaways.** Motion that depends on timing did not survive the simulator's acceleration limits and had to be closed with odometry feedback. When several behaviors share `cmd_vel`, exactly one has to own it at a time. Chaining a separate node into the state machine works as long as the handoff is explicit. The details are in [Learning Objectives and Final Takeaways](#learning-objectives-and-final-takeaways).

![How the self-designed behavior fits together: a person drives and maps, then draws or clicks a route that path_following.py follows, and the FSM watches its status.](docs/figures/pipeline.png)

*Figure 1. The self-designed behavior (Behavior 4). Blue: mapping. Purple: planning and following. The dashed line is the only connection to the state machine.*

## Individual Behaviors

### Behavior 1: Drive in a Square

**What it does.** [drive_square.py](ros_behaviors_fsm/drive_square.py) drives the Neato around a 1 m x 1 m square: forward 1 m, turn left 90 degrees, four times. It stops if something comes within 0.5 m of the front of the robot or if a message arrives on the `estop` topic, and it resumes when the condition clears.

**Implementation.** The node subscribes to `odom` (`nav_msgs/Odometry`), `scan` (`sensor_msgs/LaserScan`) and `estop` (`std_msgs/Bool`), and publishes `cmd_vel`. Each leg uses proportional control on the *remaining* distance or angle: forward speed is capped at 0.1 m/s (gain 0.5), turn rate at 0.2 rad/s (gain 1.0), and both taper toward a small minimum near the target. Progress is measured from odometry (position for legs, yaw for turns, with angle wraparound handled). After every motion the node commands zero velocity and waits one second (`settle()`) so the robot is fully stopped before the next motion starts.

**Design decisions.**

- *Odometry instead of timing.* The class's timed sample undershot in Gazebo. The Neato model's diff-drive plugin sets `max_wheel_acceleration` to 1.0 rad/s^2, so the robot needs about a second to reach the commanded speed, and a fixed sleep does not account for that.
- *Proportional control instead of a hard stop.* Commanding zero the instant the target is reached still overshoots, because the wheels also decelerate slowly. An offline simulation of the acceleration limit predicted about +8.8 degrees of overshoot per turn at constant speed, versus about -0.5 degrees with the proportional controller, which is why we ramp speed down near the target.
- *Settle pause.* Without it a turn started while the robot was still coasting forward, producing an arc instead of a pivot.
- *Multi-threaded.* The drive sequence runs on its own `threading.Thread` and blocks on short sleeps, while `rclpy.spin` handles the e-stop and scan callbacks on the main thread. That keeps the e-stop responsive without extra bookkeeping. `manual_estop` and `obstacle_close` are separate flags combined in `stopped()`, so each can clear independently and the robot can resume. (The class sample latched the stop forever; this fixes that.)

**Results.** In one Gazebo run the node reported every leg within 0.02 m of 1 m (0.98 m) and every turn within about 1 degree of 90 (89.0 to 89.2), with no segment cut short. RViz, drawing from `/odom` and TF, showed a clean square. The Gazebo 3D viewport looked wrong in the same run and we did not determine why. One caveat: the diff-drive plugin computes `/odom` from wheel motion, so it is not an independent ground truth and would not reveal wheel slip.

**Demo.** [bags/drive_square_demo](bags/drive_square_demo) (44 s). Leg 1 completed at 0.98 m (0.02 m short) and the following turn at 89.0 degrees (1.0 degree short), matching the Results above. Leg 2 was cut short by the e-stop: `Obstacle detected 0.49m ahead, stopping!`, and the node's `run_loop` simply finishes its `for` loop after that rather than resuming, so the robot stops at 0.44 m into leg 2 and the bag ends there. This is accurate behavior for a square driven in a room with a wall in the way, not a bug in the bag.

### Behavior 2: Collision Avoidance and Obstacle Avoidance

**What it does.** [collision_avoidance.py](ros_behaviors_fsm/collision_avoidance.py) combines two behaviors that the assignment lists separately: a hard e-stop, and steering around obstacles so the robot keeps moving. Bump or a reading closer than 0.3 m ahead stops the robot. Anything within 1.0 m otherwise pushes it away from the obstacle while it keeps driving.

**Implementation.** The node subscribes to `bump` and `scan` and publishes `cmd_vel` plus a `visualization_msgs/Marker` arrow (`collision_avoidance_force`) showing the net force in RViz. The hard stop looks at the closest valid reading in a +/-10 degree cone ahead, not a single ray. Otherwise it builds a potential field: a constant forward pull plus, for every scan point closer than 1.0 m, a push away from that point with magnitude `k_repulsive * (1/r - 1/1.0)`, which grows as the obstacle gets closer and is zero at 1.0 m. The heading of the summed force, `atan2(net_y, net_x)`, drives a proportional steering command clamped to 1.0 rad/s. Forward speed is 0.1 m/s scaled by `1 / (1 + |R|)`, where `|R|` is the length of the repulsion (the net force minus the constant forward pull, so open space gives the full 0.1 m/s), and drops to zero if the net force points backward.

**Design decisions.**

- *Stop and steer in one node.* A hard stop is right when something is already too close for steering to help. The field handles everything farther out, so the robot reroutes early instead of driving up to an obstacle and stopping.
- *Slowdown scales with the whole repulsion vector, not just its forward component.* An obstacle directly to one side pushes almost entirely sideways, so the forward component alone would leave the robot at full speed while passing close to it. Scaling by the vector's length covers that side-swipe case.
- *No-return readings are skipped.* The simulator reports "no return" as `inf` (in the recorded bags about 1% of readings are `inf` and none are exactly `0.0`), and the code skips any reading that is not finite or not positive, so a missing reading is never treated as an obstacle at distance zero.

**Status: three bugs found and fixed, then re-verified live in Gazebo.** The commit history records the original failure ("Collision avoidance is not working, the neato doesnt move forward"). We reproduced it offline by giving the node scans laid out like the simulator's, and found three causes:

1. *The scan is indexed from the wrong direction.* The simulator's `/scan` has 361 rays and `angle_min = -pi`, but the lidar is mounted rotated by 180 degrees in the robot model (`neato_with_camera.sdf`), so ray 0 points at the front of the robot, not ray 180. We confirmed this two ways from the recorded bags. Treating ray 0 as the front makes the hits from many scans line up on the same walls (2,395 distinct 5 cm cells, versus 6,223 when `angle_min` is applied), and while the robot drives straight forward the range at ray 180 grows by 1.00 m per meter driven. The code turned a ray index into an angle with `angle_min + i * increment`, so it treated ray 180 as the front. The hard-stop cone looked behind the robot (an obstacle 0.25 m ahead did not trigger it, and the robot kept creeping forward), and the whole force field was rotated 180 degrees (a wall on the left pushed the robot into it).
2. *The slowdown counted the forward pull as repulsion.* After the latest commit the length used for the slowdown included the constant attraction, so open space gave 0.05 m/s instead of 0.10.
3. *The repulsion is a sum over every ray, while the attraction is one constant.* With `k_repulsive = 1.0`, a wall 0.5 m to one side cut the forward speed to about 0.002 m/s, so the robot effectively did not move near walls.

**The fixes.** Ray `i` is now at angle `i * angle_increment` from the front, both in `_range_at_angle` (wrapping at 360 rays per turn, not 361) and in the force sum. The slowdown again subtracts the forward pull, and `k_repulsive` is 0.02 as a starting value. On scans laid out like the simulator's this gives 0.10 m/s in open space, a hard stop for an obstacle 0.25 m ahead, and a turn away from a wall 0.5 m to either side at 0.055 m/s. On the 480 real scans in the two bags, the hard-stop cone matched the ray-0 window every time. The FSM's own `COLLISION_AVOIDANCE` state is a simpler separate implementation; it had the same scan-indexing bug, now fixed (see the FSM section).

**Two more bugs found and fixed live in Gazebo, 2026-09-22.** A teammate reported obstacle avoidance "just rams into the object" (the topic-mismatch fix in `fsm_node.py`, above, addressed that one) and separately asked whether it steers around obstacles or just stops. Testing that directly, facing the robot at an obstacle dead-on and letting it run:

1. *A symmetric obstacle produced a maxed-out, flip-flopping turn with no forward motion.* An obstacle straight ahead is the same distance on both sides of center, so the un-rotated repulsion summed to a vector pointing almost exactly backward. `atan2(net_y, net_x)` then sits on the +/-pi discontinuity, where floating-point noise flips the sign every scan -- confirmed live: `angular.z` alternated between the +1.0 and -1.0 rad/s clamp while `linear.x` stayed 0, and the robot's true position (checked directly against the simulator, not `/odom`) did not move for 10 straight seconds. Fixed by rotating every repulsive contribution in `compute_potential_field` by a fixed 20-degree bias (`self.repulsion_bias_angle`), so a symmetric obstacle no longer produces a force sitting exactly on that discontinuity -- verified offline first (the same symmetric input now gives a consistent heading every time, across several distances, with no change to the open-space or single-sided-obstacle cases), then live (the robot turned one direction and stayed turning, instead of alternating).

   ![Left: an obstacle straight ahead produces a net force at exactly 180 degrees. Right: rotating every repulsion vector by 20 degrees moves the net force well off that point.](docs/figures/potential_field_fix.png)

   The exact mechanism, computed with the real `compute_potential_field` (not illustrative numbers): a symmetric obstacle's repulsion sums to a heading of exactly +180.0 degrees before the fix, and -137.0 degrees after.
2. *Even with that fixed, the robot could still get stuck at a distance right at the hard-stop threshold.* Steering moved it just far enough to clear the threshold, the very next scan put it right back under it, and `cmd_vel` ended up alternating between a real command and an exact `(0, 0)` on almost every single message. With the wheels' real acceleration limit (see Behavior 1), each between-stops command was too short-lived to ever build actual speed, so the robot again did not move, confirmed the same way. Fixed with hysteresis: `stop_distance_clear` (0.4 m) and `side_stop_distance_clear` (0.32 m) are now required to *release* a hard stop, wider than the 0.3 m/0.22 m that *trigger* one. Re-verified live in the same scenario: the robot moved a real 0.4 m away and its distance to the obstacle grew from 0.4 m to just over 1.0 m in 15 s, instead of staying flat.

**A third bug, found live the same way, 2026-09-22.** After the two fixes above, a real touch (not just close range) still left the robot stopped permanently, matching what a teammate reported: it does not move again until the object is moved. Root cause: the simulator's bump sensor only publishes `/bump` while something is actually touching the robot, and goes silent -- no "cleared" message at all -- once contact ends. `process_bump` only ever set `self.bumped = True`; nothing ever set it back to `False`, since no message ever arrives to do it. Confirmed live: after one real touch, `self.bumped` stayed `True` forever, even after teleporting the robot to open space with zero new `/bump` messages for the following two seconds. This is the same underlying sensor behavior already fixed once before, in `safe_teleop.py` (day 3). Fixed the same way here: track the time of the last `bump == True` message and treat it as cleared once 0.3 s has passed with no new one, instead of waiting for an explicit "false" message that never comes. Re-verified live with the identical steps: hard-stopped while genuinely touching, then resumed forward motion within the 0.3 s window after being moved to open space.

One design limit remains, not yet tested: the hard stop still has no recovery *motion* once it does hold (it corrects the *stuck-forever* bug above, but a still-blocked robot just keeps waiting and re-triggering, rather than backing away like the FSM's own `COLLISION_AVOIDANCE` state does).

**By design, this node does not move the robot when run completely alone.** It publishes to `cmd_vel_collision_avoidance`, not `cmd_vel` -- confirmed live: running `ros2 run ros_behaviors_fsm collision_avoidance` by itself in Gazebo leaves `/cmd_vel` with zero publishers. The rename feeds the gateway node, `fsm_node.py` (see [Finite State Machine](#a-second-fsm-fsm_nodepy)), which now correctly forwards it (the topic-name mismatch described below is fixed). To see this node alone, either run it with `fsm_node.py` (`ros2 launch ros_behaviors_fsm fsm.launch.py`, now installed and working -- see How To Run) or remap its output directly: `ros2 run ros_behaviors_fsm collision_avoidance --ros-args -r cmd_vel_collision_avoidance:=cmd_vel`, which is how the demo bag below was recorded.

**Demo.** [bags/collision_avoidance_demo](bags/collision_avoidance_demo) (35 s, re-recorded 2026-09-22 against the current code, remapped onto `cmd_vel` as above). The robot moved continuously and steered around obstacles for the whole run, with no bumps and a minimum approach distance of 0.22 m (the `side_stop_distance`). No bump-triggered hard stop appears in it (`/bump` has 0 messages); that path is exercised by `finite_state_controller_demo` instead.

### Behavior 3: Wall Following

**What it does.** The robot drives forward while keeping a set distance from a wall and staying parallel to it.

**Implementation.** Two implementations exist and they differ.

- [wall_follower.py](ros_behaviors_fsm/wall_follower.py) is the standalone node. It follows the closer of the two side walls (the left if only the left is visible or both are equally close, otherwise the right; it re-picks on every scan). It uses three readings on that side, at 90 degrees (distance) and 45 and 135 degrees (alignment), read directly by index (0 is the front, 90 the left, 270 the right, which matches the simulator's scan layout). `angular.z = kp * (side_reading - target) + kp * (front_diagonal - rear_diagonal)` for a left wall, and the negative of that for a right wall, with `kp = 0.5` and a 1.0 m target. If the front reading is 0.8 m or closer it stops and turns away from the wall at 0.4 rad/s; if any of the three readings is missing it drives forward at 0.1 m/s while turning slowly (0.1 rad/s) toward the wall's side to look for it.
- The FSM's `WALL_FOLLOWING` state in [finite_state_controller.py](ros_behaviors_fsm/finite_state_controller.py) handles a wall on either side. It latches which side the wall is on (`follow_side`, +1 left and -1 right) and reads the same three angles mirrored to that side. The turn is `follow_side * (kp_distance * (side - 0.4) + kp_align * (front - rear))`, clamped to 0.5 rad/s.

**Design decisions.** The alignment term (`front - rear`) is zero when the robot is parallel, and if the nose points into the wall the front diagonal shrinks, which turns the robot away. Missing readings contribute zero error instead of a garbage correction. We checked the steering directions against synthetic scans. The standalone version steers correctly on both sides (wall parallel at the target, too close, too far) and turns away from an obstacle ahead. The FSM version now does too with the simulator's real scan layout; before the scan-indexing fix described under Behavior 2 it steered the wrong way in every case we tried.

**Status.** Confirmed live in Gazebo, 2026-09-22 (see `bags/wall_follower_demo` below): placed beside a wall, the robot followed it forward while steering, and separately turned away cleanly at a corner instead of driving into it. One possible cause of the choppiness the commit history mentions ("Woks on both sides, but it is a bit choppy") that we did not test is that it re-picks the side on every scan, so it can switch walls when both are in range. It also still has unused variables (`is_turning`, `turn_start_time`, `turn_duration`, `has_teleop_run`), prints debug lines on every scan, and only filters `inf` and `nan` for the three wall readings, so a `0.0` reading would count as a wall at zero distance. The simulator does not produce `0.0` readings, but we have not checked the physical robot.

The FSM version had the scan-layout problem described under Behavior 2: with the simulator's layout it picked the opposite side and turned away from a wall that was too far and toward one that was too close, for both left and right walls. That is fixed, and on synthetic scans it now picks the right side and steers correctly on both. It has not been tuned or run in Gazebo with a real wall. Neither version publishes the wall-detection `Marker` the assignment asks for, and the two use different target distances (1.0 m versus 0.4 m), so they will not behave identically.

**By design, this node does not move the robot when run completely alone**, for the same reason as `collision_avoidance.py` above: it publishes `cmd_vel_wall_follower`, and needs either `fsm_node.py` or a remap to reach `cmd_vel`. Run together with the gateway (see [Finite State Machine](#a-second-fsm-fsm_nodepy)) it works: with no wall in range it drives forward while searching, and `fsm_node.py` correctly forwards that to `/cmd_vel`.

**Demo.** [bags/wall_follower_demo](bags/wall_follower_demo) (35 s, re-recorded 2026-09-22 against the current code, remapped onto `cmd_vel` as with collision avoidance above; started beside a wall). It followed the wall forward, and 11 of the 143 logged commands show `angular.z` near the fixed 0.4 rad/s turn speed, matching a corner turn-away, not just the proportional steering correction. It has no wall marker topic, which the assignment asks for and we did not add.

### Behavior 4: Mapping, A* Planning and Path Following (self-designed)

This is our self-designed behavior. It lets a person drive the robot around a room once to build a map, then either draw a route on the map or click a goal and have the robot plan and drive it.

**Mapping.** [teleop_scan.py](ros_behaviors_fsm/teleop_scan.py) drives the robot from the keyboard (`w/s/a/d/q/e`, space to stop, `+/-` for speed, `m` to save) while [room_map.py](ros_behaviors_fsm/room_map.py) builds an occupancy grid from the lidar. Each scan updates a log-odds grid: cells along a beam become more likely free, and the cell where it ends becomes more likely occupied. The grid is published on `room_map` for RViz and saved as a standard PGM plus YAML pair (default `~/.ros/room_map.yaml`) in the `odom` frame.

**Planning.** [a_star.py](ros_behaviors_fsm/a_star.py) plans over that saved map. Occupied cells are first inflated by the robot's radius, so a path keeps the whole robot clear of walls and not just its center. A* then searches 8-connected cells (diagonal steps cost sqrt(2)) using straight-line distance as the estimate, and returns waypoints in world coordinates. In offline tests on synthetic maps it routes through a doorway wide enough for the robot, refuses one that is too narrow, and returns nothing when the goal is walled off. With the follower's 0.28 m safety radius (6 cells at 0.05 m per cell) a doorway has to be about 0.7 m wide: in a synthetic test 0.5 m was refused and 0.7 m was accepted. A start point inside the inflated zone (within 0.28 m of a wall) also returns no path, even though the robot is standing there.

**Following.** [path_following.py](ros_behaviors_fsm/path_following.py) opens a Tkinter window showing the map. Drag with the left button to draw a route, then press *Go*; or right-click a goal and press *Plan (A\*)*. If a hand-drawn route crosses an obstacle, the node falls back to A* toward the same destination instead of refusing. Waypoints are resampled (0.15 m spacing) and smoothed, then followed with pure pursuit: aim at the point 0.3 m ahead along the path, steer proportionally (gain 1.5), turn in place when the heading error exceeds 50 degrees, and slow down near the goal. It pauses on a bump, an e-stop, or anything within 0.25 m ahead, and resumes when clear. It publishes the path on `drawn_path` and its state (`idle`, `following`, `paused`, `done`) on `path_following_status`.

**Design decisions.**

- *Pure pursuit* was chosen for its simplicity and because it tolerates the small heading errors from a differential-drive robot without needing a full trajectory controller.
- *Inflating obstacles once, before the search,* keeps the search itself cheap. The alternative, checking the robot's footprint at every visited cell, would repeat the same work many times.
- *Threading.* `rclpy.spin` runs on a background thread here, because Tkinter's `mainloop` (and, in `teleop_scan.py`, the blocking keyboard loop) need the main thread. This is the reverse of `drive_square.py`, where the work is threaded off and ROS stays on the main thread.

**Localization (in progress).** [icp_localizer.py](ros_behaviors_fsm/icp_localizer.py) and [icp_matching.py](ros_behaviors_fsm/icp_matching.py) correct odometry drift by matching the live scan against the saved map. `icp_align` starts from the odometry-based pose and repeats up to 20 times: place the scan on the map, pair each scan point with its nearest occupied map point (dropping pairs over 0.5 m apart), then solve for the shift and rotation that best line the pairs up (a closed-form 2D solution, no SVD). The localizer only moves halfway toward the ICP answer (`gain = 0.5`) and rejects a match if too few points matched, the error is large, or the jump is large. It publishes `localized_pose`, the `map -> odom` transform, and `localization_confident`, a `Bool` that is true when at least 4 of the last 5 scans passed those checks.

It was tested offline on a synthetic room with a simulated lidar, and end to end with the node against a saved map of that room. It recovered the pose to about 1 mm with a 1 cm map and about 3 cm with a 5 cm map (roughly half a map cell), converges from a starting error of up to about 0.4 m and 0.3 rad, and `localization_confident` went false on scans from the wrong place and true again afterwards. It has not been run on a real map from Gazebo or the physical Neato, and the FSM does not use `localization_confident` yet.

**Testing.** A* was tested offline as described above. The mapping and path-following nodes were run in Gazebo and recorded: [bags/teleop_scan_demo](bags/teleop_scan_demo) (40 s, includes `/room_map` and `/scan`) and [bags/path_following_demo](bags/path_following_demo) (56 s, includes `/drawn_path`, `/path_following_status` and `/cmd_vel`). No bump events occurred in either recording, so the pause-on-obstacle behavior is not shown in them. Play them back with `ros2 bag play bags/<name> --clock`.

![Mapping run: the occupancy grid with the robot's path, and the keyboard commands over time.](docs/figures/teleop_scan_demo.png)

*Figure 2. `teleop_scan_demo`. Left: the final `/room_map` (dark = occupied, white = free, grey = unknown) with the `/odom` path colored by time. Right: the `/cmd_vel` commands. The keyboard only sends fixed steps, 0 or 0.15 m/s forward and turns of 0.6 rad/s.*

![Path following run: two paths drawn over the map with the robot's actual path, the commands sent, and the node's status over time.](docs/figures/path_following_demo.png)

*Figure 3. `path_following_demo`. Two paths were sent (dashed) and the robot followed both (blue). Right: `/cmd_vel`, with the purple bands marking when `/path_following_status` was `following`, and the status over time below. The map comes from `teleop_scan_demo`; it lines up here because both bags start at the same odometry origin, as the run instructions require.*

What the data shows:

- Both paths finished (`done`). The robot stopped about 0.06 m from the last waypoint of path 1 and about 0.04 m from the last waypoint of path 2, inside the 0.08 m goal tolerance. The status never became `paused`.
- Around t = 14 to 17 s the robot is at the far end of path 1 (x of about 1.5 m), where the route doubles back. Linear speed falls to about 0.06 m/s and angular speed climbs to its 1.0 rad/s cap. That is the rule that slows the robot as heading error grows and turns it in place past 50 degrees.
- Path 2 starts (t = 36 s) with a pure turn, linear 0 and angular +1.0 rad/s, before the robot drives off. Computing it from the odometry, the robot faced about -91 degrees while the point 0.3 m ahead on the path was at about -3 degrees, a heading error of about 88 degrees. That is past the 50 degree threshold, so it turned in place until the error dropped below it (about 0.7 s later) and then started driving.
- The commands are jumpy while following, with brief dips in linear speed. We did not investigate the cause or smooth them, and the robot still completed both paths.

## Finite State Machine

### Overall Design

**Intended performance.** When the FSM runs, the Neato drives a 1 m square. While it is driving the square, if it bumps something or sees an obstacle less than 0.3 m ahead, it backs away while turning toward the side with more room, then goes back to the square once nothing is in the way. If a wall comes within 0.75 m on either side, it follows that wall until the wall is gone, then returns to the square. If someone starts a path in `path_following.py`, the FSM stops sending velocity commands so that node can drive, and takes over again with the square once the path is finished or cancelled. After the fourth turn of the square the robot stops and waits.

![Finite state machine: DRIVE_SQUARE is the default state; bump or an obstacle leads to COLLISION_AVOIDANCE, a nearby wall leads to WALL_FOLLOWING, and the path node's status leads to PATH_FOLLOWING.](docs/figures/fsm.png)

*Figure 4. The state machine. Solid arrows leave `DRIVE_SQUARE`, dashed arrows return to it. The purple `PATH_FOLLOWING` arrow is checked first on every tick and applies from any state.*

| State | What the robot does |
|---|---|
| `DRIVE_SQUARE` | Drives a 1 m square with timed legs (5 s forward, 2 s turn), then stops after the fourth turn. |
| `COLLISION_AVOIDANCE` | Backs away at 0.05 m/s while turning (0.3 rad/s) toward whichever front diagonal (+/-45 degrees) has more clearance. |
| `WALL_FOLLOWING` | Follows the latched wall using the steering law in Behavior 3. |
| `PATH_FOLLOWING` | Publishes nothing; `path_following.py` is driving. |

**Transitions.** Bump comes from the `bump` topic and obstacle detection from the closest reading in a +/-10 degree cone ahead (under 0.3 m). Wall detection is a reading under 0.75 m at +/-90 degrees. The `PATH_FOLLOWING` check runs first on every tick, so it preempts whatever state the FSM is in, and it can only return to `DRIVE_SQUARE`. The two reactive states never connect to each other; both return to `DRIVE_SQUARE` once their trigger clears.

### Implementation Details

`finite_state_controller.py` is one node with a `State` enum, a `self.state` variable, and a 10 Hz timer calling `run_loop`. Sensor callbacks (`process_bump`, `process_scan`, `process_path_following_status`) only update variables. Each tick, `run_loop` first picks the next state from those variables, then calls one handler (`handle_drive_square`, `handle_collision_avoidance`, `handle_wall_following` or `handle_path_following`) that publishes the velocity for that state.

**Coding strategy.** The assignment lists three ways to build an FSM: reimplement behaviors in one node, use libraries, or chain parallel nodes. We used two of them. `DRIVE_SQUARE`, `COLLISION_AVOIDANCE` and `WALL_FOLLOWING` are reimplemented inline, as simplified versions of the standalone nodes above (not calls into them). `PATH_FOLLOWING` chains a separate node: `path_following.py` keeps its own `cmd_vel` publisher and safety logic and reports its state on `path_following_status`, and while that state is active the FSM's handler deliberately publishes nothing. We chose this because `path_following.py` owns a GUI, a saved map and its own pause logic, and two nodes reacting to the same sensors and publishing conflicting `cmd_vel` messages would be worse than either alone.

### Capabilities and Limitations

- **Scan indexing (fixed, and now confirmed live).** `_range_at_angle` used to convert an angle to a ray index using `angle_min`, but the simulator's front is ray 0 (see Behavior 2), so the FSM's obstacle, wall and clearance readings were 180 degrees off: an obstacle 0.2 m ahead did not leave `DRIVE_SQUARE`, one 0.2 m behind sent it to `COLLISION_AVOIDANCE`, a left wall was treated as a right wall, and `WALL_FOLLOWING` steered the wrong way. It now indexes from the front. With scans laid out like the simulator's, an obstacle ahead goes to `COLLISION_AVOIDANCE`, one behind is ignored, the wall side is right, and wall steering is correct on both sides. The transition logic itself already behaved as in Figure 4 offline (we stepped through 11 transitions; see What We Verified); the live run in Demonstration confirms it also holds with real scans, at least until the wedging failure above.
- The transition into `PATH_FOLLOWING` is not triggered by anything the robot senses. It starts when a person presses *Go* or *Plan (A\*)* in the GUI, which the FSM only sees through the status topic. The assignment asks for transitions that can be detected in the environment, so we note this as a real limitation of the design.
- The FSM's `DRIVE_SQUARE` is the timed version, so it inherits the drift described in Behavior 1 (the odometry-based `drive_square.py` is not used inside the FSM), and after the fourth turn it just idles.
- The standalone behavior nodes (`drive_square`, `collision_avoidance`, `wall_follower`) publish `cmd_vel` themselves and must not run at the same time as the FSM or each other.
- `follow_side` is only cleared when no wall is visible on either side, so it can stay latched to a wall that has disappeared while the other side still triggers detection. Missing readings contribute zero error, which keeps this from producing a wild command, but the robot may not steer toward the wall that is actually there.
- Bump and obstacle are only checked while in `DRIVE_SQUARE`. While the robot is in `WALL_FOLLOWING` nothing stops it from driving into something ahead, because that state has no transition to `COLLISION_AVOIDANCE`.
- **Bump latching (fixed, 2026-09-22).** The simulator's bump sensor only publishes while something is touching the robot, and `simulator_adapter` only republishes `/bump` when it receives one of those messages, so `/bump` never reports that contact has ended. `process_bump` here (and in `path_following.py`) used to just assign `self.bumped` from the message fields, so once a real bump arrived it stayed `True` forever, keeping the FSM stuck in `COLLISION_AVOIDANCE` (or `path_following.py` stuck `paused`) with no way back -- the same root cause found and fixed first in `collision_avoidance.py` (Behavior 2) and, before that, in `safe_teleop.py`. Fixed the same way in both files: `process_bump` now only ever sets `self.bumped = True` and records the time, and a new `check_bump_timeout()` (called every tick from `run_loop` / `control_loop`, not just from a scan or bump callback) clears it after 0.3 s with no new bump. Verified directly against the real classes: for `finite_state_controller.py`, publishing one `/bump` message live pinned `/cmd_vel` at all-zero, and it resumed real motion (nonzero `angular.z`, `DRIVE_SQUARE` turning) once the timeout elapsed with no further bump messages. For `path_following.py`, a synthetic test driving the real `PathFollower` class (mid-path, `state="following"`) showed one bump call flipping it to `paused`, and `check_bump_timeout()` alone flipping it back to `following` after 0.3 s.
- **Wedging (found 2026-09-22, not fixed).** `handle_collision_avoidance`'s back-away-and-turn recovery can drive the robot into a spot in `gauntlet_harmonic.world` it cannot escape -- confirmed live, reproduced identically in two independent runs (see Demonstration). Nothing in the FSM checks odometry against real progress to detect this.
- The FSM has been run end to end in the simulator (see Demonstration), including real transitions between `DRIVE_SQUARE`, `COLLISION_AVOIDANCE` and `WALL_FOLLOWING`. The handoff to `PATH_FOLLOWING` specifically is still only checked with synthetic inputs (see What We Verified), not live.

### Demonstration

[bags/finite_state_controller_demo](bags/finite_state_controller_demo) (65 s, from spawn, 2026-09-22). Live, and separately in two fresh, independent 90 s runs used to write this section: the robot travelled about 9.5 m of real distance in the first roughly 45 s, driving the square, backing off from obstacles and following walls -- clear evidence the local state machine is doing real work, not just idling in `DRIVE_SQUARE`.

**A wedging failure mode, found live, 2026-09-22.** In both of those 90 s runs the robot then went physically motionless for the remaining time, at the same true position both times (checked against the simulator's ground truth, not `/odom`: `(2.03, -3.06)`). The front and rear scan readings were exactly unchanged for the rest of each run (`0.26 m` / `1.59 m`), which confirms the robot genuinely was not moving relative to its surroundings -- this is not a sensor dropout. `/cmd_vel` kept publishing the same nonzero command (`linear.x = 0.1`, `angular.z = -0.08`) the whole time, so the wheels were still being commanded to turn. Meanwhile `/odom` kept changing by a large amount (from about `(2.0, -3.9)` to `(-0.3, -3.1)`, a swing of over a meter, across identical resets), which is odometry lying under wheel slip -- the same failure mode documented earlier this session for `teleop_scan.py` when the robot is pinned against something. `COLLISION_AVOIDANCE`'s backing-and-turning maneuver appears able to drive the robot into a spot in this particular room (`gauntlet_harmonic.world`) that it cannot back back out of, and nothing in the FSM cross-checks odometry against actual progress to detect that. We did not fix this; it is a real, reproducible limitation of `handle_collision_avoidance`'s recovery motion, not of the transition logic itself (see Capabilities and Limitations).

### A second FSM: `fsm_node.py`

A teammate built a second, different state-machine design partway through the project: [fsm_node.py](ros_behaviors_fsm/fsm_node.py) and [launch/fsm.launch.py](launch/fsm.launch.py). It is registered in `setup.py` alongside `finite_state_controller.py`, as a separate `fsm_node` executable. **The two have not been reconciled into one FSM and both are still in the repository; this project has not decided which one it submits, or documented that decision. That is the one item in this section still genuinely open** -- everything else found in it (listed below) has since been fixed and re-verified live.

![fsm_node.py's gateway architecture: wall_follower.py, collision_avoidance.py, drive_square.py and path_following.py each publish to their own cmd_vel_* topic, fsm_node.py forwards whichever one matches its current state to cmd_vel, and current_mode tells path_following.py when to show its GUI.](docs/figures/fsm_node.png)

*Figure 5. `fsm_node.py`'s gateway design, contrasted with Figure 4's single-node design. Green notes mark items fixed and re-verified live; the amber notes mark the two limitations that are inherent to the design (the keyboard listener under non-interactive stdin, and `drive_square.py` running once on startup rather than on state entry) and were not changed.*

**The design is different: a topic-mux gateway instead of one node with sensor callbacks.** `wall_follower.py`, `collision_avoidance.py`, `drive_square.py` and `path_following.py` each publish `Twist` on their own topic instead of `cmd_vel`. `fsm_node.py` subscribes to all of those, plus `cmd_vel_teleop_scan` and the raw `scan`, and republishes whichever one matches its current state (`"WALL FOLLOW"`, `"OBSTACLE AVOIDANCE"`, `"TELEOP SCAN"`, `"DRIVE SQUARE"`, or `"PATH FOLLOWING"`) to `cmd_vel`. It also publishes its current state on `current_mode`, which `path_following.py` uses to show or hide its own GUI. The state switches automatically on the closest scan reading in a +/-10 ray cone (`OBSTACLE AVOIDANCE` under 0.5 m, back to `WALL FOLLOW` over 0.7 m -- see the stuck-timeout fix below), or manually with `t` (teleop scan), `m` (drive square, once teleop has run), `g` (wall follow) or `p` (path following, once teleop has run) on the keyboard. This explains the topic renames in `wall_follower.py` and `collision_avoidance.py` described under Behaviors 2 and 3 -- they were made to feed this gateway, not a mistake on their own.

- `ros2 launch ros_behaviors_fsm fsm.launch.py`, exactly as its own file implies it should be run, fails immediately. The launch file was never added to `setup.py`'s `data_files`, so it is not installed anywhere `ros2 launch` looks. It also gives each of the three nodes a different package name (`wall_follower`, `collision_avoidance`, `fsm_node`); all three are executables inside the single `ros_behaviors_fsm` package, so even an installed copy would still fail to find those packages.
- Running the three nodes by hand instead, wall following worked: with no wall in range it drove forward while turning to search, and `fsm_node.py` correctly forwarded that command to `cmd_vel`, and the robot moved.
- **Fixed, 2026-09-21, later the same evening.** `collision_avoidance.py` published `cmd_vel_collision_avoidance`; `fsm_node.py` subscribed to a different name, `cmd_vel_obstacle_avoidance`, so nothing was ever forwarded -- confirmed directly with `ros2 topic info` on both, and this is what a teammate saw as "obstacle avoidance doesn't work, it just rams into the object." Changed `fsm_node.py`'s subscription to `cmd_vel_collision_avoidance` to match what the node actually publishes. Re-verified live: `/cmd_vel_collision_avoidance` now shows 1 publisher and 1 subscriber (was 1 and 0), and a drive-into-a-cube test with all three nodes running stopped the robot about 0.33 m short with no bump, though in that run the stop came from `wall_follower.py`'s own front-distance check (it turns away from anything within 1.0 m on its own) rather than from `"OBSTACLE AVOIDANCE"` actually triggering.
- **`"OBSTACLE AVOIDANCE"` firing end to end, confirmed 2026-09-22.** The run above concluded the state hadn't fired because `fsm_node`'s printed state stayed `"WALL FOLLOW"` the whole time -- that conclusion was itself an artifact: Python block-buffers `print()` once stdout is redirected to a file (as our test harness did), so the log we were tailing was stale, not real-time. Confirmed by calling the real `FSMNode.run_loop` directly with a synthetic close-range scan (state flipped `WALL FOLLOW` -> `OBSTACLE AVOIDANCE` -> `WALL FOLLOW` exactly as designed) and, separately, live: feeding a synthetic `/scan` with an obstacle at 0.3 m changed `/cmd_vel`'s `angular.z` from wall-following's small proportional correction to a large, continuously-valued turn (not `wall_follower.py`'s fixed +-0.4 rad/s turn-in-place), which only `collision_avoidance.py`'s potential-field steering produces -- and it reverted to the small wall-following correction once the fake obstacle cleared. `"OBSTACLE AVOIDANCE"` does fire and forward correctly.
- **A separate, more serious problem found while trying to confirm the above, fixed 2026-09-22.** Every scan/camera/IMU-based test this session (this one included) had actually been running against a Gazebo world with no lidar data at all: `maze.world` (what `neato_maze.py`, and therefore this project's whole "Quick start", launches) never declared the `gz::sim::systems::Sensors` or `...::Contact` system plugins, so gz-sim silently fell back to its bare-minimum default set (`Physics`, `UserCommands`, `SceneBroadcaster` only) and `/scan`, `/imu`, `/camera/*` and `/bump` never published anything -- confirmed directly (`gz topic -i -t /scan` -> "No publishers"), and confirmed the fix by comparing against `gauntlet_final.world`, which already has the correct plugin block and was the world actually used for the earlier live cube tests in this write-up (that's why those worked while this one didn't). `flatland.world`, `dh.world` and `bod_volcano.world` are missing the same plugins and are presumably equally broken, though we didn't test them. Added the same plugin block to `maze.world`; re-verified `/scan` publishing real ranges immediately after.
- **There is no bump subscription anywhere in `fsm_node.py`**, so state switching here only ever looks at `/scan`'s front cone, never at `/bump`. `collision_avoidance.py` itself still hard-stops correctly on a real bump (its own `self.bumped`/`self.too_close` logic, Behavior 2, is unaffected by any of this), but `fsm_node.py` only forwards that stop to `/cmd_vel` while its own state is `"OBSTACLE AVOIDANCE"`. A bump the front-cone scan check didn't already catch (e.g. a side touch, or contact in a scan blind spot) would leave the state at `"WALL FOLLOW"`, so `wall_follower.py`'s command keeps going to the wheels instead of the stop -- the robot could keep driving despite `collision_avoidance.py` correctly computing zero.
- The keyboard listener that `t`/`a` depend on runs `termios.tcgetattr(sys.stdin)` in a background thread, which raises immediately if stdin is not an interactive terminal. We hit exactly this running it from a script; it would happen the same way under `ros2 launch`, or in any background/non-interactive process. The thread dies silently (it is a daemon thread), the node keeps running in the state it was last in, and the manual switch stops working.
**Bugs found and fixed, 2026-09-21 to 2026-09-22, each confirmed live in Gazebo:**

1. **Topic-name mismatch (fixed 2026-09-21).** `collision_avoidance.py` published `cmd_vel_collision_avoidance`; `fsm_node.py` subscribed to a different name, `cmd_vel_obstacle_avoidance`, so nothing was ever forwarded -- confirmed directly with `ros2 topic info` on both, and this is what a teammate saw as "obstacle avoidance doesn't work, it just rams into the object." Fixed by changing `fsm_node.py`'s subscription to match. Re-verified: `/cmd_vel_collision_avoidance` went from 1 publisher/0 subscribers to 1/1.
2. **`"OBSTACLE AVOIDANCE"` firing end to end, confirmed 2026-09-22.** An earlier test seemed to show the state never changing, but that was an artifact of Python block-buffering `print()` once stdout is redirected to a file. Confirmed for real: feeding a synthetic `/scan` with an obstacle at 0.3 m changed `/cmd_vel`'s `angular.z` from wall-following's small proportional correction to `collision_avoidance.py`'s large, continuously-valued potential-field turn, and it reverted once the fake obstacle cleared.
3. **Launch file did not run at all (fixed 2026-09-22).** `ros2 launch ros_behaviors_fsm fsm.launch.py` failed immediately: the file was never added to `setup.py`'s `data_files`, so it was not installed anywhere `ros2 launch` looks, and it named three packages that do not exist (`wall_follower`, `collision_avoidance`, `fsm_node` -- all three are executables inside one package, `ros_behaviors_fsm`). Fixed by moving it to a conventional `launch/` directory, installing that directory via `setup.py`, and pointing every `Node` at the real package name. Added [launch/bringup.launch.py](launch/bringup.launch.py), which also starts Gazebo first, for a true one-command start. Re-verified live: both launch files find every node with no errors (the keyboard-listener limitation below is unrelated to the launch mechanism itself).
4. **Stuck in `"WALL FOLLOW"` forever near a corner (fixed 2026-09-22).** `wall_follower.py` turns away on its own once something is within 1.0 m, which kept the front reading just above `fsm_node.py`'s 0.5 m `"OBSTACLE AVOIDANCE"` threshold indefinitely -- confirmed live: placed near a corner and run for 45 s, `fsm_node`'s printed state was `"WALL FOLLOW"` on all 224 scan callbacks, and the robot visibly spun in place instead of escaping. Fixed with a timeout tracked independent of the current state: if the front reading has stayed inside `wall_follower.py`'s own 1.0 m reactive zone for more than 3 s without clearing past 1.3 m, `fsm_node.py` forces `"OBSTACLE AVOIDANCE"` regardless of the 0.5 m absolute check, and stays there (even past a momentary nudge over the 0.7 m recovery line) until genuinely clear. Re-verified live, twice, from a fresh reset each time: 90 s runs with 6 clean transitions between the two states, 0 bumps, and 7.46 to 7.50 m travelled (the earlier, unfixed version travelled 1.67 to 1.83 m in the same time and never escaped a corner it got stuck near). [bags/fsm_node_demo](bags/fsm_node_demo) (95 s) is one of these runs, recorded with `wall_follower.py`, `collision_avoidance.py` and `fsm_node.py` running together; it shows 6 `current_mode` messages, matching the 6 transitions.

**Two limitations found live that we did not change, because they are inherent to the current design rather than bugs in it:**

- The keyboard listener that `t`/`m`/`g`/`p` depend on runs `termios.tcgetattr(sys.stdin)` in a background thread, which raises immediately if stdin is not an interactive terminal (true under `ros2 launch`, or any background process). The thread dies silently (it is a daemon thread); the node keeps running in whatever state it was last in, but the manual switch stops working. `teleop_scan.py` detects this itself and exits cleanly with a clear error instead; `fsm_node.py` does not.
- `drive_square.py` runs its four-leg sequence once, immediately when the node starts, regardless of `fsm_node.py`'s state -- it is not re-triggered on entering `"DRIVE SQUARE"`. If the FSM is not already in that state by the time the sequence finishes, `cmd_vel_drive_square` has nothing left to forward, and pressing `m` afterward does nothing. Under `bringup.launch.py`, which starts in `"WALL FOLLOW"`, this means `drive_square.py`'s motion is effectively unreachable through the gateway; run `ros2 run ros_behaviors_fsm drive_square` by itself to see it drive.
- There is still no bump subscription anywhere in `fsm_node.py`, so nothing here reacts to a physical bump the way `finite_state_controller.py`'s `bumped` flag does.

None of the above made it into "What We Verified" as failures invented to find fault -- they are the direct, reproducible result of running the files as committed, all now fixed except the two noted as inherent limitations and the still-open question of which FSM this project submits.

## What We Verified

**First pass, 2026-09-21 (earlier that day):** synthetic inputs and the two recorded bags only. Nothing was run in Gazebo.

| Check | How | Result |
|---|---|---|
| Build and entry points | `colcon build`, then import each of the 7 registered nodes | Builds; all 7 import and have a `main()` |
| FSM transitions | Stepped `run_loop` through 11 events (obstacle, wall, bump, and path status `following`, `paused`, `done`) using scans with the front at ray 0 | Every step went to the state in Figure 4; `PATH_FOLLOWING` published nothing |
| FSM with the simulator's scan layout | Same node, scans laid out like the simulator's | Before the scan-indexing fix: obstacle ahead missed, obstacle behind detected, wall side swapped, wall steering reversed. After: obstacle ahead detected, obstacle behind ignored, wall side and steering correct on both sides |
| Standalone wall follower | Synthetic scans: left and right wall at, closer than, and farther than the target; obstacle ahead | Steering direction correct in every case |
| Standalone collision avoidance | Synthetic scans laid out like the simulator's | Before the fix: fails as described in Behavior 2. After: 0.10 m/s in open space, hard stop for an obstacle 0.25 m ahead, turns away from a wall 0.5 m to either side at 0.055 m/s, straight down a corridor |
| Collision avoidance on the recorded scans | Ran the fixed hard-stop cone against all 480 scans in the two bags | Cone matched the ray-0 window on every scan; no hard stops; forward speed above zero on 419 of 480 |
| Scan layout and no-return value | Two tests on the recorded bags (see Behavior 2) | Front is ray 0; no-return is `inf`, never `0.0` |
| A* | Synthetic map with a wall and a doorway, radius 0.28 m | Doorways of 0.1 and 0.5 m: no path. 0.7 and 1.0 m: path of 51 points. Sealed wall: none. Start inside the inflated zone: none |
| ICP | Synthetic room, 5 cm map, start guesses up to 0.25 m and 0.2 rad off | Pose error 2.1 to 2.4 cm and 0.6 to 1.3 degrees |
| Pure-pursuit controller | Closed loop with an ideal unicycle model (no acceleration limit): straight line, half circle, L-shaped path | Reached the goal within 4.6 to 7.3 cm (tolerance 8 cm) in 18 to 35 s |
| Docs | Checked every local link and figure in both documents | All exist |
| Style tests | `pytest test/` | `test_flake8` (152 issues at the time) and `test_pep257` (137 issues) fail; `test_copyright` is skipped. They do not affect the robot. **Re-checked 2026-09-22, after more commits: 445 and 170 respectively -- see the third pass below.** |

**Second pass, 2026-09-21 (same evening, after the bags were recorded and a teammate pushed `fsm_node.py`):** live in Gazebo this time, against the code on disk at the time of each test.

| Check | How | Result |
|---|---|---|
| `finite_state_controller.py`, `COLLISION_AVOIDANCE` | Placed the robot facing a 0.5 m cube (`cube_20k_1`) with a 1 m runway, ran the node, logged the true simulator pose every second for 12 s | Drove forward for about 2 s, then backed away while turning at the obstacle, matching `handle_collision_avoidance`. **This is the first live Gazebo confirmation of any FSM transition.** |
| `collision_avoidance` and `wall_follower`, run exactly as the "How To Run" section says | `ros2 run ros_behaviors_fsm collision_avoidance` (then `wall_follower`), each alone, and checked `ros2 topic info /cmd_vel` while it ran | **Zero publishers on `/cmd_vel` for either.** Both nodes run without error but currently publish to `cmd_vel_collision_avoidance` and `cmd_vel_wall_follower`, which nothing in the simulator subscribes to. The robot does not move. This is not what Behaviors 2 and 3 currently say. |
| Why the recorded bags look fine despite that | Diffed the installed executable against the source right after the bags were recorded | The installed `collision_avoidance` still published to `/cmd_vel` -- the rename to `cmd_vel_collision_avoidance` was already committed, but `colcon build` had not been re-run before recording. The bags show a build that no longer exists on disk. |
| `fsm_node.py` + `fsm.launch.py` (new, from `f02ca4e`/`ae49987`, the author's message says "only halfway there") | `ros2 launch ros_behaviors_fsm fsm.launch.py` | Fails immediately: the file was never added to `setup.py`'s `data_files`, so it is not installed under `share/ros_behaviors_fsm/`. It also names `wall_follower`, `collision_avoidance` and `fsm_node` as three separate packages; all three executables actually live in `ros_behaviors_fsm`. |
| Same three nodes, started by hand instead of through the launch file (before the fix below) | `ros2 run` all three (`wall_follower`, `collision_avoidance`, `fsm_node`) together, watched `/cmd_vel` and the simulator pose for 10 s | Wall following's output reached `/cmd_vel` through the gateway and the robot moved. Collision avoidance's never could: `fsm_node.py` subscribed to `cmd_vel_obstacle_avoidance`, but `collision_avoidance.py` publishes to `cmd_vel_collision_avoidance` -- confirmed directly, `ros2 topic info` showed 0 subscribers on the one and 0 publishers on the other. In this 10 s run the robot curved away from the cube while searching for a wall and never got close enough to trigger `OBSTACLE AVOIDANCE`, so we could not also observe the robot failing to stop; the topic mismatch alone was enough to know it would have ignored an obstacle if that state had been reached. |
| `fsm_node.py`'s `cmd_vel_obstacle_avoidance`/`cmd_vel_collision_avoidance` mismatch, after fixing it | Changed `fsm_node.py`'s subscription to `cmd_vel_collision_avoidance`, rebuilt, reran the same three-node test facing the cube | `/cmd_vel_collision_avoidance` now shows 1 publisher and 1 subscriber. The robot stopped about 0.33 m from the cube with no `/bump` messages, though the stop we observed came from `wall_follower.py`'s own front-distance check, not a confirmed `OBSTACLE AVOIDANCE` firing -- `fsm_node`'s printed state stayed `"WALL FOLLOW"` for the whole run. |
| `fsm_node.py`'s keyboard listener (`t`/`a` to switch mode) | Ran the node with stdin that is not an interactive terminal (true for anything started through `ros2 launch`, and for our own test) | The listener thread crashes immediately (`termios.error: (25, 'Inappropriate ioctl for device')`). It is a daemon thread, so the node itself keeps running in the initial `"WALL FOLLOW"` state, but the manual mode switch this design depends on is unavailable. |
| Why `maze.world` (the "Quick start" world) never showed a lidar-triggered transition, and correcting the row above, 2026-09-22 | `gz topic -i -t /scan` while `neato_maze.py` was running | **No publishers on topic [/scan]** -- `maze.world` never declared the `Sensors`/`Contact` system plugins gz-sim needs for `/scan`, `/imu`, `/camera/*` and `/bump`, so it silently used the bare 3-plugin default (`Physics`, `UserCommands`, `SceneBroadcaster`) instead. The earlier `cube_20k_1` tests above happened to work because `cube_20k_1` only exists in `gauntlet_final.world`, which already has the right plugins. Fixed by adding the same plugin block to `maze.world`; `/scan` published real ranges immediately after. |
| `"OBSTACLE AVOIDANCE"` firing end to end, re-tested after the world-file fix, 2026-09-22 | Called the real `FSMNode.run_loop` directly with a synthetic close-range scan; separately, live, fed a synthetic `/scan` through the topic while `wall_follower`, `collision_avoidance` and `fsm_node` all ran normally | Direct call: state went `WALL FOLLOW` -> `OBSTACLE AVOIDANCE` -> `WALL FOLLOW` exactly as designed. Live: `/cmd_vel`'s `angular.z` switched from wall-following's small proportional correction to a large, continuously-valued turn (not `wall_follower.py`'s fixed +-0.4 rad/s), which only `collision_avoidance.py`'s steering produces, then reverted once the fake obstacle cleared. The row above's "still open" conclusion was a stdout-buffering artifact (see Behavior 2/FSM sections), not a real gap -- the state does fire and forward correctly. |

**Third pass, 2026-09-22:** live in Gazebo, after the fixes above and several more teammate commits.

| Check | How | Result |
|---|---|---|
| `finite_state_controller.py`, full run | Fresh Gazebo reset, ran the node for 90 s from spawn, twice, logging the true simulator pose every 10 s | The robot travelled about 9.5 m of real distance, driving the square, backing off from obstacles and following walls -- confirming real `DRIVE_SQUARE`/`COLLISION_AVOIDANCE`/`WALL_FOLLOWING` transitions with live scans, not just the offline-synthetic check from the first pass. In both runs it then got physically wedged at the same true position (a limitation, not a crash -- see the FSM's Demonstration section) |
| `launch/fsm.launch.py`, after moving it into `launch/` and adding it to `setup.py`'s `data_files` | `ros2 launch ros_behaviors_fsm fsm.launch.py` | Found and launched all six nodes with no errors (aside from the keyboard-listener limitation under non-interactive stdin, unrelated to the launch mechanism) |
| `launch/bringup.launch.py` (new) | `ros2 launch ros_behaviors_fsm bringup.launch.py` from a clean state, no Gazebo running yet | Started the gauntlet world and every app node with one command; `/odom` and `/scan` came up within a few seconds |
| `fsm_node.py`'s stuck-in-`"WALL FOLLOW"` bug and its fix | Placed near a corner, ran `fsm_node.py` + `wall_follower.py` + `collision_avoidance.py` for 45 s (before) and twice for 90 s from a fresh reset (after) | Before: state stayed `"WALL FOLLOW"` for all 224 scan callbacks, robot visibly stuck spinning. After: 6 clean transitions per 90 s run, 0 bumps, 7.46-7.50 m travelled, robot visibly escapes the corner instead of spinning -- see [bags/fsm_node_demo](bags/fsm_node_demo) |
| `collision_avoidance.py` and `wall_follower.py`, run alone with a topic remap | `ros2 run ros_behaviors_fsm collision_avoidance --ros-args -r cmd_vel_collision_avoidance:=cmd_vel` (and the equivalent for `wall_follower.py`) | Both moved the robot continuously with the remap; used to re-record `bags/collision_avoidance_demo` and `bags/wall_follower_demo` against the current code |
| Style tests, re-checked | `pytest test/` | `test_flake8`: 445 issues (up from 152), mostly a new quote-style split (523 `Q000` findings) between files a teammate's editor reformatted to double quotes and the project's original single-quote convention, concentrated in `path_following.py` (138) and `teleop_scan.py` (64). `test_pep257`: 170 issues (up from 137), mostly docstrings recently added in an imperative-mood style pep257 does not accept ("Processes the scan..." instead of "Process the scan..."). None of this affects the robot; we did not clean it up given the time this took to even characterize -- see Status |

**Not checked:** the FSM's `PATH_FOLLOWING` state live (only checked with synthetic inputs), the `icp_localizer` node against a real Gazebo or physical-robot map, the path-following GUI driven by a real mouse (only synthetic events), the teleop keyboard loop driven by real key presses (only scripted input), and the physical robot (nothing in this project has been run on it).

## Team Contributions

**TODO before submitting: the middle column below is every author `git log` shows for each file (checked again 2026-09-22), in commit order, so several files have more than one name. It shows who touched the code, not who designed or debugged which part. Confirm it and fill in the last column.**

| Person | Files committed (from git history, most files have more than one author) | Other contributions |
|---|---|---|
| Aditi | `angle_helpers.py`, `a_star.py`, `README.md`, `WRITEUP.md`; also committed to `drive_square.py`, `collision_avoidance.py`, `finite_state_controller.py`, `fsm_node.py`, `wall_follower.py`, `path_following.py`, `icp_localizer.py`, `icp_matching.py`, `setup.py`, `package.xml` | TODO |
| Duc | `room_map.py`; also committed to `collision_avoidance.py`, `fsm_node.py`, `teleop_scan.py`, `path_following.py`, `icp_localizer.py`, `icp_matching.py`, `setup.py`, `package.xml`; wrote the ICP stub implementations (`find_correspondences`, `best_rigid_transform`, `icp_align`) in `icp_matching.py`; fixed `fsm_node.py`'s stuck-in-`"WALL FOLLOW"` bug and installed `launch/fsm.launch.py` correctly, and added `launch/bringup.launch.py`; recorded all seven bags in `bags/` | TODO |
| Akil | `fsm_node.py`, `launch/fsm.launch.py` (originally `fsm.launch.py`), `drive_square.py`; also committed to `collision_avoidance.py`, `wall_follower.py`, `finite_state_controller.py`, `teleop_scan.py`, `path_following.py`, `setup.py` | TODO |

## Learning Objectives and Final Takeaways

**Individual learning objectives.** TODO: one short paragraph each on what you set out to learn and what you learned (for example, what you now understand about control loops, state machines, mapping or planning that you did not before).

- Aditi: TODO
- Duc: TODO
- Akil: TODO

**Challenges.**

- Timed motion did not survive contact with the simulator's acceleration limits. Finding that took logging target versus actual for every leg (the printed lines showed errors of a few centimeters and about a degree once it was fixed).
- The first several explanations for "the square looks wrong" were about our controller. The controller was fine by odometry and RViz; what looked wrong was the Gazebo viewport. We never established why, which is a reminder that `/odom` in simulation is derived from the same wheel model and is not independent ground truth.
- Behaviors written separately can collide. Several nodes publishing `cmd_vel` at once is easy to create by accident and needs an explicit handoff.
- The simulator's scan layout is not what its header suggests. `/scan` says `angle_min = -pi` and has 361 rays, but the lidar is mounted rotated 180 degrees, so ray 0 is the front. Code that indexed from the front (drive square, wall follower, mapping, path following) works; code that trusted `angle_min` (collision avoidance and the FSM) looked backward until we fixed it. We only found this at the end, because our synthetic test scans used a different layout. We had also assumed missing readings show up as `0.0`; in the simulator they are `inf`.
- A rebuild is not automatic. `collision_avoidance_demo` and `wall_follower_demo` were recorded after `collision_avoidance.py` and `wall_follower.py` were changed to publish to a different topic, but `colcon build` was not re-run first, so the recording used the previous behavior and looked correct. Checking the installed executable's actual content, not just the source file, is what caught it.
- Renaming a topic to prepare for a not-yet-finished consumer breaks the previous consumer immediately. Once `collision_avoidance.py` and `wall_follower.py` stopped publishing `cmd_vel` (to feed the new `fsm_node.py` gateway), running either one alone -- exactly as our own "How To Run" instructions say to at the time -- stopped moving the robot at all, and the gateway they were renamed for was not finished either. A brief note in a commit message or a shared channel would have caught this sooner.
- A distance threshold alone can make a state unreachable if another behavior's own reaction keeps the triggering reading just on the safe side of it forever. `fsm_node.py`'s `"OBSTACLE AVOIDANCE"` and `finite_state_controller.py`'s wedging failure are two different symptoms of the same underlying issue: a purely reactive check has no memory of how long it has been almost triggering, or of whether the robot is actually making progress. A timeout (for the first) or a progress check against ground truth (still needed for the second) are both ways of adding that memory back in.
- A quick reformat by one editor and a hand-written style by another do not mix quietly. The flake8/pep257 counts roughly tripled between two points in this project without any functional change, from a docstring-and-quote-style pass on a few files that the project's test suite (correctly) flags as now internally inconsistent.

**What we would do with more time.**

- **Reconcile `finite_state_controller.py` and `fsm_node.py` into one FSM.** Two independent designs exist and both work; picking one (or merging their strengths -- the single-node design's bump handling, the gateway's clean per-behavior process isolation) is the largest remaining item.
- Fix `finite_state_controller.py`'s wedging failure mode (Demonstration): have the recovery motion notice when it isn't making real progress, not just react to the current scan.
- Tune the gains in `collision_avoidance.py` and the wall-following states against real runs, and unify the two wall-following implementations (they use different target distances, 1.0 m versus 0.4 m).
- Run the ICP localizer against a real map from Gazebo and the physical Neato, use the corrected pose in path following, and use `localization_confident` as a sensed trigger in an FSM.
- Add a sensed trigger for path following (for example, starting it when a goal is published on a topic) instead of a person pressing a button.
- Add the wall-detection `Marker`.
- Normalize quote style and docstring mood across the files touched by more than one author, to bring the style-test counts back down.
- Test everything on the physical Neato.

**Takeaways.**

1. *Close the loop when the actuator has dynamics.* If the plant accelerates and decelerates slowly, open-loop timing fails silently. Measuring progress and tapering speed fixed what tuning the timing could not.
2. *Check a surprising result against a second source before changing code.* The viewport, RViz and the controller's own log disagreed, and comparing them told us which one to distrust.
3. *Give every actuator one owner at a time.* When independently written behaviors share `cmd_vel`, decide explicitly who is in control, as the FSM does with `path_following_status`.
4. *Test the math offline with synthetic data, but build the test inputs from a recording.* The A* planner and the steering signs were checked against fake maps and fake scans without waiting for a simulator run, which caught real mistakes. But our first fake scans had a different layout from the simulator's, so they passed code that fails on the real data. Check a sensor's layout against a recorded bag before writing the tests.
5. *Chaining nodes is a legitimate FSM design.* When a behavior's architecture (a GUI, a map, its own safety logic) does not fit the others, handing off to it beats reimplementing it, as long as the handoff is clean.
6. *A purely reactive transition can be permanently unreachable, not just wrong.* `fsm_node.py`'s obstacle-avoidance switch was correct on paper (a distance threshold) but never actually fired, because another behavior's own reaction kept the input just on the safe side of it. Watching how long a condition has been almost-true, not just whether it is true right now, closed that gap without changing anything else about the design.

## How To Run

Prerequisites: ROS 2 Jazzy, the `neato_packages` workspace (`neato2_gazebo`, `neato2_interfaces`, ...), `numpy`, `pyyaml` and Tkinter.

1. Download the code into a workspace's `src/` folder (skip the first clone if you already have the class's Neato packages), then build and source:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/comprobo26/neato_packages.git
   git clone https://github.com/aditilagisetty/CompRobo-FSM-Project.git
   cd ~/ros2_ws
   colcon build --packages-select ros_behaviors_fsm
   source install/setup.bash
   ```
2. **Easiest: start everything with one command**, which starts the gauntlet world and the whole `fsm_node.py` app (`wall_follower`, `collision_avoidance`, `drive_square`, `path_following`, `teleop_scan`):
   ```bash
   ros2 launch ros_behaviors_fsm bringup.launch.py             # pass world:=maze / empty / gauntlet_final / bod for a different world
   ```
   With no keyboard input the FSM starts in `"WALL FOLLOW"` and reacts on its own; press `t`/`m`/`g`/`p` in the terminal running `fsm_node` to switch modes by hand (see [A second FSM](#a-second-fsm-fsm_nodepy) -- this needs a real terminal, and `drive_square.py`'s motion is only reachable by running it separately, as below). To bring up just the app's nodes against a world you already started, run `ros2 launch ros_behaviors_fsm fsm.launch.py` instead.
3. **Or start a world and run one behavior at a time**, in separate terminals:
   ```bash
   ros2 launch neato2_gazebo neato_gauntlet_world.py   # or empty_world.py, neato_maze.py
   ```
   ```bash
   ros2 run ros_behaviors_fsm drive_square              # publishes cmd_vel directly -- moves the robot
   ros2 run ros_behaviors_fsm finite_state_controller   # publishes cmd_vel directly -- moves the robot
   ros2 run ros_behaviors_fsm collision_avoidance       # publishes cmd_vel_collision_avoidance -- add the remap below to move the robot alone
   ros2 run ros_behaviors_fsm wall_follower             # publishes cmd_vel_wall_follower -- add the remap below to move the robot alone
   ```
   The last two feed `fsm_node.py`'s gateway by design (see [A second FSM](#a-second-fsm-fsm_nodepy)); to see either move the robot by itself without the gateway, remap its output directly, e.g. `ros2 run ros_behaviors_fsm collision_avoidance --ros-args -r cmd_vel_collision_avoidance:=cmd_vel`. Do not run two `cmd_vel`-publishing nodes (or two nodes both remapped onto `cmd_vel`) at once.
4. Mapping and path following:
   ```bash
   ros2 run ros_behaviors_fsm teleop_scan       # drive around, press m to save, Ctrl-C to quit
   ```
   The map is stored in the odometry frame, so restart the simulator so the robot begins at the same pose it had when you scanned, then:
   ```bash
   ros2 run ros_behaviors_fsm path_following    # left-drag to draw + Go, or right-click a goal + Plan (A*)
   ```
   To see the FSM hand off to path following, run `finite_state_controller` in a third terminal alongside it. (Under `bringup.launch.py`/`fsm.launch.py` instead, `path_following.py`'s GUI opens and closes on its own via `current_mode` when you press `p`/another key in `fsm_node`'s terminal.)
5. Record a demo, and play one back (use `--clock`, and disconnect from the robot first):
   ```bash
   ros2 bag record /accel /bump /odom /cmd_vel /scan /stable_scan /projected_stable_scan /tf /tf_static /clock -o bags/<name>
   ros2 bag play bags/<name> --clock
   ```
   Each bag is a folder inside `bags/`. To see one, start RViz (`rviz2`) in another terminal and add displays for `/room_map`, `/drawn_path`, `/scan` or `/tf`.
6. Rebuild the graphs in this write-up (needs ROS sourced; matplotlib for the bag-based and potential-field figures, Graphviz's `dot` for the three diagrams):
   ```bash
   python3 docs/make_figures.py
   python3 docs/plot_potential_field_fix.py
   dot -Tpng -Gdpi=200 docs/fsm.dot -o docs/figures/fsm.png
   dot -Tpng -Gdpi=200 docs/pipeline.dot -o docs/figures/pipeline.png
   dot -Tpng -Gdpi=200 docs/fsm_node.dot -o docs/figures/fsm_node.png
   ```

## Repository Contents

| Path | Purpose |
|---|---|
| `ros_behaviors_fsm/drive_square.py` | Odometry-based square driving with e-stop |
| `ros_behaviors_fsm/collision_avoidance.py` | Hard stop plus potential-field steering |
| `ros_behaviors_fsm/wall_follower.py` | Standalone wall follower (left or right wall) |
| `ros_behaviors_fsm/finite_state_controller.py` | One state machine: sensor callbacks plus per-state handlers in a single node |
| `ros_behaviors_fsm/fsm_node.py` | A second state machine: a topic-mux gateway. Not reconciled with the file above (see What We Verified) |
| `ros_behaviors_fsm/teleop_scan.py`, `room_map.py` | Keyboard driving and occupancy-grid mapping |
| `ros_behaviors_fsm/a_star.py` | A* planner with obstacle inflation |
| `ros_behaviors_fsm/path_following.py` | Path GUI, planning entry points, pure-pursuit follower |
| `ros_behaviors_fsm/icp_localizer.py`, `icp_matching.py` | ICP localization against the saved map (tested on synthetic data, not yet on a real map) |
| `ros_behaviors_fsm/angle_helpers.py` | Quaternion to Euler conversion |
| `launch/fsm.launch.py` | Brings up every app node (`fsm_node`, `wall_follower`, `collision_avoidance`, `path_following`, `teleop_scan`, `drive_square`) against a world already running |
| `launch/bringup.launch.py` | `fsm.launch.py` plus a `neato2_gazebo` world first, for a one-command start |
| `bags/` | Seven recorded runs, one per node/behavior: `teleop_scan_demo`, `path_following_demo`, `drive_square_demo`, `collision_avoidance_demo`, `wall_follower_demo`, `finite_state_controller_demo`, `fsm_node_demo` |
| `docs/` | Diagram sources (`fsm.dot`, `pipeline.dot`, `fsm_node.dot`), `make_figures.py` (bag-based figures), `plot_potential_field_fix.py`, and the figures in `docs/figures/` used in this write-up |

## Status (delete before submitting)

Still open at the time of writing (2026-09-22, evening):

- [x] Fix the scan indexing in `collision_avoidance.py` and `finite_state_controller.py`, the slowdown, and the `k_repulsive` scale; re-verified live in Gazebo, not just on synthetic scans (see What We Verified, second and third pass).
- [x] Bags recorded for all seven nodes/behaviors: `drive_square_demo`, `collision_avoidance_demo`, `wall_follower_demo`, `finite_state_controller_demo`, `path_following_demo`, `teleop_scan_demo`, `fsm_node_demo`. `collision_avoidance_demo` and `wall_follower_demo` were re-recorded 2026-09-22 against the current code (remapped onto `cmd_vel`, as How To Run describes) and now match it.
- [x] `collision_avoidance.py` and `wall_follower.py` publish `cmd_vel_collision_avoidance`/`cmd_vel_wall_follower` by design, to feed `fsm_node.py`'s gateway. "How To Run" now documents both the gateway and the remap needed to see either move the robot completely alone.
- [x] **`fsm_node.py`'s launch file now runs.** Moved to `launch/fsm.launch.py`, installed via `setup.py`'s `data_files`, and pointed at the real package name (`ros_behaviors_fsm`) for all nodes. Added `launch/bringup.launch.py`, which also starts Gazebo. Both verified live with no errors.
- [x] `fsm_node.py` subscribed to `cmd_vel_obstacle_avoidance`, but `collision_avoidance.py` publishes `cmd_vel_collision_avoidance` -- fixed and re-verified live; `"OBSTACLE AVOIDANCE"` firing end to end is also now separately confirmed (see A second FSM).
- [x] `compute_potential_field` produced a maxed-out, flip-flopping turn with no forward motion for an obstacle straight ahead. Fixed with a fixed 20-degree bias on every repulsive vector; re-verified offline and live.
- [x] The robot could get stuck oscillating right at the hard-stop threshold. Fixed with hysteresis; re-verified live.
- [x] `collision_avoidance.py`'s `self.bumped` stayed `True` forever after one real touch. Fixed with a timeout; re-verified live.
- [x] `finite_state_controller.py` and `path_following.py` had the same `self.bumped`-latches-forever bug. Fixed with the same timeout approach; re-verified live/synthetically.
- [x] **`fsm_node.py` stuck in `"WALL FOLLOW"` forever near a corner** (found 2026-09-22): `wall_follower.py`'s own 1.0 m turn-away kept the front reading just above the 0.5 m switch threshold indefinitely. Fixed with a 3 s stuck timeout tracked independent of state; re-verified live twice (6 clean transitions, 0 bumps, 7.5 m travelled per 90 s run -- see A second FSM).
- [ ] **`fsm_node.py` has no bump subscription.** Decide whether that state machine needs one, or document that bump handling is `finite_state_controller.py`-only. (Inherent to the design, not fixed.)
- [ ] **`fsm_node.py`'s keyboard listener crashes when stdin is not an interactive terminal**, which is the normal case under `ros2 launch` or in the background. The daemon thread dies silently and the manual mode switch stops working; the node itself keeps running. (Inherent to the design, not fixed.)
- [ ] **`drive_square.py` runs its square once, immediately on startup, not gated by `fsm_node.py`'s state** -- found 2026-09-22 while writing the launch file. Its motion is effectively unreachable through `bringup.launch.py`/`fsm.launch.py` unless the FSM is already in `"DRIVE SQUARE"` by the time it finishes; run it separately to see it drive. (Inherent to the current design, not fixed.)
- [ ] **`finite_state_controller.py`'s `COLLISION_AVOIDANCE` recovery can wedge the robot** in a spot in `gauntlet_harmonic.world` it cannot back out of -- found live 2026-09-22, reproduced identically in two independent 90 s runs (see the FSM's Demonstration section). Not fixed; would need the recovery motion to notice when it isn't making real progress.
- [ ] **Two FSMs now exist** (`finite_state_controller.py` and `fsm_node.py`) and have not been reconciled. **This is now the single biggest open item** -- decide which one this project submits, or how they relate, before the write-up's FSM section is final.
- [ ] Wall-detection `Marker` (required by the assignment for wall following). Neither `wall_follower.py` nor the FSM's `WALL_FOLLOWING` state publishes one.
- [ ] Tune `k_repulsive`, `k_steer` and the wall-following gains against real runs.
- [ ] **Style-test failures got worse, not better: `test_flake8` is now 445 issues (was 152) and `test_pep257` is 170 (was 137)**, mostly a quote-style split introduced when a teammate's editor reformatted some files to double quotes against the project's original single-quote convention (523 of the 445... -- flake8 double-counts some lines under multiple checks; see What We Verified, third pass, for the exact breakdown) plus newly added docstrings not in pep257's required imperative mood. We characterized this but did not clean it up given the time it would take across files with more than one author; a `--diff`-scoped `autopep8`/quote-normalization pass per file, confirmed with `pytest test/` and a rebuild after, would be the fastest way to close most of it without hand-editing.
- [x] Run `finite_state_controller.py`'s `COLLISION_AVOIDANCE` state, `drive_square.py`, `collision_avoidance.py`, `wall_follower.py`, and the full `finite_state_controller.py` FSM live in Gazebo (see What We Verified). `WALL_FOLLOWING` fires live as part of the full-FSM run; `PATH_FOLLOWING` is still only checked with synthetic inputs.
- [ ] Test on the physical Neato. Nothing in this project has been.
- [ ] Add gifs, video or bag graphs for `drive_square_demo`, `wall_follower_demo`, `finite_state_controller_demo` and `fsm_node_demo` (`teleop_scan_demo` and `path_following_demo` already have figures via `docs/make_figures.py`; `collision_avoidance.py`'s discontinuity fix already has one). After recording/re-recording a bag, add it to a `figure_*` function in `docs/make_figures.py`.
- [ ] Confirm the Team Contributions table (it is a draft from git history) and fill in each person's other contributions.
- [ ] **Write each person's individual learning objectives** -- these have to be each person's own words; nobody else can write them accurately.
- [ ] Confirm the ICP description with whoever is working on it. Since this was last written, the three math stubs (`find_correspondences`, `best_rigid_transform`, `icp_align`) have been filled in; the description of what they do should be checked against the actual implementation before this line is deleted.
