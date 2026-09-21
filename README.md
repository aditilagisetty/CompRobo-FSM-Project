# RoboBehaviors and Finite State Machines Project

Aditi, Duc and Akil. Olin ENGR3590 Computational Introduction to Robotics.
Assignment: [RoboBehaviors and FSMs](https://comprobo26.github.io/assignments/warmup_project).

A ROS 2 Jazzy package (`ros_behaviors_fsm`) that drives a Neato in Gazebo with four behaviors and a finite state machine that combines them. **The full write-up, with design decisions, figures, limitations and how to run everything, is in [WRITEUP.md](WRITEUP.md).**

![State machine](docs/figures/fsm.png)

## Where things stand

| Behavior | File | Status |
|---|---|---|
| Drive a 1 m square | `drive_square.py` | Works in Gazebo: every leg within 0.02 m and every turn within about 1 degree (odometry feedback). |
| Collision avoidance | `collision_avoidance.py` | Scan-indexing and speed bugs fixed and checked on synthetic and recorded scans; **not re-run in Gazebo yet**, gains untuned. The hard stop has no recovery motion. Details in the write-up. |
| Wall following | `wall_follower.py`, and the `WALL_FOLLOWING` state | Standalone node steers correctly on both sides (reported choppy in the sim). The FSM's version had the same scan-indexing bug, now fixed (not re-run in Gazebo). No wall `Marker` yet. |
| Mapping, A* planning, path following (self-designed) | `teleop_scan.py`, `room_map.py`, `a_star.py`, `path_following.py` | Run in Gazebo and recorded in `bags/`. ICP localization (`icp_localizer.py`) is tested on synthetic data only. |
| State machine | `finite_state_controller.py` | Transition logic checked with synthetic inputs; never run end to end in the simulator. Its scan reading was fixed to match the sim's layout (see the write-up). |

Nothing has been tested on the physical Neato.

## Quick start

```bash
cd ~/ros2_ws
colcon build --packages-select ros_behaviors_fsm
source install/setup.bash
ros2 launch neato2_gazebo neato_maze.py            # in its own terminal
ros2 run ros_behaviors_fsm drive_square            # one behavior at a time; each publishes cmd_vel
```

Other nodes: `collision_avoidance`, `wall_follower`, `finite_state_controller`, `teleop_scan`, `path_following`, `icp_localizer`. Mapping and path-following steps, and how to record and play back bags, are in the write-up's [How To Run](WRITEUP.md#how-to-run).

## Layout

| Path | Contents |
|---|---|
| `ros_behaviors_fsm/` | The ROS nodes and their helpers |
| `bags/` | Recorded runs: `teleop_scan_demo`, `path_following_demo` |
| `docs/` | Diagram sources, `make_figures.py`, and the figures used in the write-up |
| `WRITEUP.md` | The write-up |

## Still to do

See the checklist at the end of [WRITEUP.md](WRITEUP.md#status-delete-before-submitting). The main items are re-running collision avoidance and the FSM in the simulator and tuning them, recording the missing bags (drive square, collision avoidance, wall following, FSM), the wall-detection `Marker`, and physical-robot testing.
