# RoboBehaviors and Finite State Machines Project

Aditi, Duc and Akil. Olin ENGR3590 Computational Introduction to Robotics.
Assignment: [RoboBehaviors and FSMs](https://comprobo26.github.io/assignments/warmup_project).

A ROS 2 Jazzy package (`ros_behaviors_fsm`) that drives a Neato in Gazebo with four behaviors and two finite state machines that combine them. **The full write-up, with design decisions, figures, limitations and how to run everything, is in [WRITEUP.md](WRITEUP.md).**

![State machine](docs/figures/fsm.png)

## Where things stand

Every node below has been run live in Gazebo, and each is demonstrated by a bag in `bags/`. Details, numbers and exact test procedures are in the write-up's [What We Verified](WRITEUP.md#what-we-verified).

| Behavior | File | Status |
|---|---|---|
| Drive a 1 m square | `drive_square.py` | Works: every leg within 0.02 m and every turn within about 1 degree (odometry feedback). |
| Collision avoidance | `collision_avoidance.py` | Steers around obstacles and hard-stops for close ones, verified live with no bumps over multiple runs. Publishes `cmd_vel_collision_avoidance`, not `cmd_vel`, by design (see Quick start). |
| Wall following | `wall_follower.py`, and the FSM `WALL_FOLLOWING` states | Standalone node steers correctly on both sides and turns away at corners, confirmed live. No wall-detection `Marker` yet (the assignment asks for one). |
| Mapping, A* planning, path following (self-designed) | `teleop_scan.py`, `room_map.py`, `a_star.py`, `path_following.py` | Run in Gazebo and recorded in `bags/`, matching real position. ICP localization (`icp_localizer.py`, `icp_matching.py`) works against a synthetic map; not yet run against a real one. |
| State machine 1: `finite_state_controller.py` | One node, sensor callbacks plus per-state handlers | Runs end to end live: drives the square, avoids obstacles, follows walls. Has a known failure mode where its obstacle-avoidance recovery can wedge the robot in a corner it can't back out of (see the write-up). |
| State machine 2: `fsm_node.py` | A topic-mux gateway; `wall_follower`/`collision_avoidance`/`drive_square`/`path_following`/`teleop_scan` each publish their own `cmd_vel_*`, this node forwards whichever matches the current state | Runs end to end live: 90 s test runs travel 7.5 m with 6 clean state transitions and no bumps. Its own launch file (`launch/fsm.launch.py`) now works. |

**The two state machines have not been reconciled.** This is the single biggest thing left to decide before submitting -- see [Status](WRITEUP.md#status-delete-before-submitting) in the write-up. Nothing has been tested on the physical Neato.

## Quick start

The easiest way to see the whole app, Gazebo included:

```bash
cd ~/ros2_ws
colcon build --packages-select ros_behaviors_fsm
source install/setup.bash
ros2 launch ros_behaviors_fsm bringup.launch.py
```

That starts the gauntlet world and `fsm_node.py`'s gateway with every behavior wired in. It reacts on its own (wall following, switching to obstacle avoidance); press `t`/`m`/`g`/`p` in the `fsm_node` terminal to switch modes by hand. To run one node at a time instead (start a world first, then one of these per terminal):

```bash
ros2 launch neato2_gazebo neato_gauntlet_world.py
ros2 run ros_behaviors_fsm drive_square              # moves the robot directly
ros2 run ros_behaviors_fsm finite_state_controller   # moves the robot directly, the other FSM
ros2 run ros_behaviors_fsm collision_avoidance --ros-args -r cmd_vel_collision_avoidance:=cmd_vel
ros2 run ros_behaviors_fsm wall_follower --ros-args -r cmd_vel_wall_follower:=cmd_vel
```

The last two publish to their own `cmd_vel_*` topic by design, to feed `fsm_node.py`'s gateway -- the `-r` remap above is only needed to see either move the robot completely alone. Mapping, path following, recording and playing back bags, and rebuilding the figures are all in the write-up's [How To Run](WRITEUP.md#how-to-run).

## Layout

| Path | Contents |
|---|---|
| `ros_behaviors_fsm/` | The ROS nodes and their helpers |
| `launch/` | `fsm.launch.py` (the app's nodes) and `bringup.launch.py` (that plus Gazebo) |
| `bags/` | Seven recorded runs, one per node/behavior |
| `docs/` | Diagram sources (`fsm.dot`, `pipeline.dot`, `fsm_node.dot`), `make_figures.py`, `plot_potential_field_fix.py`, and the figures used in the write-up |
| `WRITEUP.md` | The write-up |

## Still to do

See the checklist at the end of [WRITEUP.md](WRITEUP.md#status-delete-before-submitting). The main items: decide how `finite_state_controller.py` and `fsm_node.py` relate, or which one this project submits; fix `finite_state_controller.py`'s corner-wedging failure mode; add the wall-detection `Marker`; tune the gains; clean up the style-test failures (`pytest test/`, currently 445 `flake8` and 170 `pep257` issues, mostly a quote-style split between files edited by different people); write each person's individual learning objectives; and physical-robot testing.
